function [delta, a_des] = mpc_combined(state, ref, veh, dt_sim, p, idx_hint, window, steer_buffer, lon_buffer)
% MPC_COMBINED  Combined lateral + longitudinal MPC controller.
%
%   [delta, a_des] = mpc_combined(state, ref, veh, dt_sim, p, ...)
%
%   State vector x = [ey; epsi; vy; r; ev; z_lon]  (6 states)
%   Control vector u = [delta; a_des]               (2 inputs)
%
%   Outputs:
%     delta  - steering angle (rad)
%     a_des  - desired acceleration (m/s^2)

    x_pos = state.x;
    y_pos = state.y;
    yaw   = state.yaw;
    vx    = max(get_vx(state), 0.5);
    vy    = state.vy;
    r_yaw = state.r;
    delta_prev = state.delta;

    if nargin < 6, idx_hint = []; end
    if nargin < 7, window = []; end
    if nargin < 8, steer_buffer = []; end
    if nargin < 9, lon_buffer = []; end

    % ── Reference point ───────────────────────────────────────────────
    [idx, xr, yr, psi_ref, ~, seg_idx, seg_t] = nearest_path_ref_point( ...
        x_pos, y_pos, ref.x, ref.y, idx_hint, window);

    kappa0 = interp_kappa(ref.kappa, idx, seg_idx, seg_t);

    dx = x_pos - xr;
    dy = y_pos - yr;
    e_y   = -sin(psi_ref) * dx + cos(psi_ref) * dy;
    e_psi = angle_wrap(yaw - psi_ref);

    % Speed error
    v_ref_now = ref.v_ref(max(min(idx, numel(ref.v_ref)), 1));
    e_v = v_ref_now - vx;

    % Longitudinal deviation (accumulated externally, passed in state)
    if isfield(state, 'z_lon')
        z_lon = state.z_lon;
    else
        z_lon = 0;
    end

    % ── Vehicle parameters ────────────────────────────────────────────
    Cf = veh.tire.front.Calpha;
    Cr = veh.tire.rear.Calpha;
    M  = veh.M;
    Iz = veh.Iz;
    lf = veh.lf;
    lr = veh.lr;
    L  = veh.L;

    % ── Dimensions ────────────────────────────────────────────────────
    nx = 6;   % [ey, epsi, vy, r, ev, z_lon]
    nu = 2;   % [delta, a_des]
    N  = p.N;
    dt = p.dt_ctrl;   % 0.1 s (10 Hz)

    % ── Continuous-time state-space ───────────────────────────────────
    A_c = zeros(nx, nx);

    % Lateral block (rows 1-4)
    A_c(1,2) = vx;    A_c(1,3) = 1;
    A_c(2,4) = 1;
    A_c(3,3) = -(Cf + Cr) / (M * vx);
    A_c(3,4) = (-lf*Cf + lr*Cr) / (M * vx) - vx;
    A_c(4,3) = (-lf*Cf + lr*Cr) / (Iz * vx);
    A_c(4,4) = -(lf^2*Cf + lr^2*Cr) / (Iz * vx);

    % Longitudinal block (rows 5-6)
    A_c(6,5) = 1;   % z_lon integrates speed error

    % B matrix
    B_c = zeros(nx, nu);
    B_c(3,1) = Cf / M;        % vy affected by delta
    B_c(4,1) = lf * Cf / Iz;  % r affected by delta
    B_c(5,2) = -1;            % ev_dot = -a_des

    % Disturbance 1: curvature (affects e_psi)
    g_c_curve = zeros(nx, 1);
    g_c_curve(2) = -vx;
    
    % Disturbance 2: reference speed derivative (affects e_v)
    g_c_dvref = zeros(nx, 1);
    g_c_dvref(5) = 1;   % e_v_dot gets +dv_ref/dt

    % ── Exact ZOH discretisation for curvature ────────────────────────
    M_aug_curve = zeros(nx + nu + 1, nx + nu + 1);
    M_aug_curve(1:nx, 1:nx)       = A_c;
    M_aug_curve(1:nx, nx+1:nx+nu) = B_c;
    M_aug_curve(1:nx, nx+nu+1)    = g_c_curve;
    E_curve = expm(M_aug_curve * dt);
    A_d = E_curve(1:nx, 1:nx);
    B_d = E_curve(1:nx, nx+1:nx+nu);
    g_curve = E_curve(1:nx, nx+nu+1);

    % ── Exact ZOH discretisation for dv_ref/dt ────────────────────────
    M_aug_dvref = zeros(nx + nu + 1, nx + nu + 1);
    M_aug_dvref(1:nx, 1:nx)       = A_c;
    M_aug_dvref(1:nx, nx+1:nx+nu) = B_c;
    M_aug_dvref(1:nx, nx+nu+1)    = g_c_dvref;
    E_dvref = expm(M_aug_dvref * dt);
    g_dvref = E_dvref(1:nx, nx+nu+1);

    % ── Curvature and speed preview ───────────────────────────────────
    n_ref = numel(ref.kappa);
    i_lo = max(1, idx - 1);
    i_hi = min(n_ref, idx + 1);
    ds_local = hypot(ref.x(i_hi) - ref.x(i_lo), ref.y(i_hi) - ref.y(i_lo)) / max(i_hi - i_lo, 1);
    ds_local = max(ds_local, 0.01);
    idx_per_step = max(1, round(vx * dt / ds_local));

    kappa_pred = zeros(N, 1);
    for k = 1:N
        fidx = min(idx + k * idx_per_step, n_ref);
        kappa_pred(k) = ref.kappa(fidx);
    end

    % ── Preview reference speed and its derivative ────────────────────
    v_ref_pred = zeros(N, 1);
    dv_ref_pred = zeros(N, 1);
    for k = 1:N
        fidx = min(idx + k * idx_per_step, n_ref);
        v_ref_pred(k) = ref.v_ref(fidx);
        if k == 1
            dv_ref_pred(k) = (v_ref_pred(k) - v_ref_now) / dt;
        else
            dv_ref_pred(k) = (v_ref_pred(k) - v_ref_pred(k-1)) / dt;
        end
    end

    % ── Delay compensation ────────────────────────────────────────────
    x0 = [e_y; e_psi; vy; r_yaw; e_v; z_lon];

    n_delay_s = 0;
    if ~isempty(steer_buffer) && numel(steer_buffer) > 1
        n_delay_s = numel(steer_buffer) - 1;
    end
    n_delay_l = 0;
    if ~isempty(lon_buffer) && numel(lon_buffer) > 1
        n_delay_l = numel(lon_buffer) - 1;
    end

    a_des_prev = 0;
    if max(n_delay_s, n_delay_l) > 0
        for d = 1:max(n_delay_s, n_delay_l)
            u_pend = [delta_prev; 0];
            if d <= n_delay_s, u_pend(1) = steer_buffer(d); end
            if d <= n_delay_l, u_pend(2) = lon_buffer(d); end
            % dv_ref/dt during delay is unknown, use 0 (conservative)
            g_d = g_curve * kappa0 + g_dvref * 0;
            x0 = A_d * x0 + B_d * u_pend + g_d;
        end
        if n_delay_s > 0, delta_prev = steer_buffer(n_delay_s); end
        if n_delay_l > 0, a_des_prev = lon_buffer(n_delay_l); end
    end

    % ── Build QP prediction matrices ──────────────────────────────────
    Qbar = kron(eye(N), p.Q);
    Rbar = kron(eye(N), p.R);

    Sx = zeros(nx*N, nx);
    Su = zeros(nx*N, nu*N);
    Sg = zeros(nx*N, 1);
    A_pow = eye(nx);

    for ii = 1:N
        A_pow = A_pow * A_d;
        rows = (ii-1)*nx+1 : ii*nx;
        Sx(rows, :) = A_pow;
        g_sum = zeros(nx, 1);
        for j = 1:ii
            A_ij = A_d^(ii-j);
            Su(rows, (j-1)*nu+1 : j*nu) = A_ij * B_d;
            g_sum = g_sum + A_ij * (g_curve * kappa_pred(j) + g_dvref * dv_ref_pred(j));
        end
        Sg(rows) = g_sum;
    end

    % ── Rate penalty: D operates on interleaved u = [d1;a1;d2;a2;...] ─
    D_single = eye(N) - [zeros(1,N); eye(N-1,N)];  % (N x N)
    D = kron(D_single, eye(nu));                     % (nu*N x nu*N)

    % Build block-diagonal Rd: kron(eye(N), Rd_2x2)
    Rd_full = kron(eye(N), p.Rd);                    % (nu*N x nu*N)

    % Previous control for rate penalty initialisation
    u_prev = [delta_prev; a_des_prev];
    d0 = zeros(nu*N, 1);
    d0(1:nu) = u_prev;

    % Free response
    x_free = Sx * x0 + Sg;

    % QP cost
    H = Su' * Qbar * Su + Rbar + D' * Rd_full * D;
    H = 0.5 * (H + H');

    f = Su' * Qbar * x_free - D' * Rd_full * d0;

    % Feedforward steering (curvature)
    delta_ff = p.kappa_ff_gain * atan(L * kappa0);

    % ── Constraints ───────────────────────────────────────────────────
    lb = zeros(nu*N, 1);
    ub = zeros(nu*N, 1);
    for k = 1:N
        id = (k-1)*nu + 1;
        ia = (k-1)*nu + 2;
        lb(id) = -p.max_steer - delta_ff;
        ub(id) =  p.max_steer - delta_ff;
        lb(ia) = p.a_min;
        ub(ia) = p.a_max;
    end

    % ── Solve ─────────────────────────────────────────────────────────
    try
        opts = optimoptions('quadprog', 'Display', 'off');
        U = quadprog(2*H, 2*f, [], [], [], [], lb, ub, [], opts);
        if isempty(U)
            delta_corr = 0;
            a_des = 0;
        else
            delta_corr = U(1);
            a_des = U(2);
        end
    catch
        delta_corr = -0.8*e_y - 1.2*e_psi - 0.05*vy - 0.08*r_yaw;
        a_des = 2.0*e_v;
    end

    delta = delta_corr + delta_ff;
    a_des = min(max(a_des, p.a_min), p.a_max);
end

function kappa0 = interp_kappa(kappa_ref, idx, seg_idx, seg_t)
    n = numel(kappa_ref);
    if ~isnan(seg_idx) && seg_idx >= 1 && seg_idx < n
        kappa0 = (1-seg_t)*kappa_ref(seg_idx) + seg_t*kappa_ref(seg_idx+1);
    else
        kappa0 = kappa_ref(idx);
    end
end

function vx = get_vx(state)
    if isfield(state, 'vx'), vx = state.vx; else, vx = state.v; end
end