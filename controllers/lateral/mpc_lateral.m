function delta = mpc_lateral(state, ref, veh, dt, p, idx_hint, window, steer_buffer)
    x = state.x;
    y = state.y;
    yaw = state.yaw;
    vx = max(get_vx(state), 0.5);
    vy = state.vy;
    r = state.r;
    delta_prev = state.delta;

    if nargin < 6
        idx_hint = [];
    end
    if nargin < 7
        window = [];
    end
    if nargin < 8
        steer_buffer = [];
    end

    % Use projected reference point instead of nearest waypoint only
    [idx, xr, yr, psi_ref, ey_proj, seg_idx, seg_t] = nearest_path_ref_point( ...
        x, y, ref.x, ref.y, idx_hint, window);

    kappa0 = interpolate_projected_curvature(ref.kappa, idx, seg_idx, seg_t);

    dx = x - xr;
    dy = y - yr;
    e_y = -sin(psi_ref) * dx + cos(psi_ref) * dy;
    e_psi = angle_wrap(yaw - psi_ref);
    

    Cf = veh.tire.front.Calpha;
    Cr = veh.tire.rear.Calpha;
    M = veh.M;
    Iz = veh.Iz;
    lf = veh.lf;
    lr = veh.lr;
    L = veh.L;

    % Dynamic bicycle error-state model:
    % x = [e_y; e_psi; v_y; r]
    x0 = [e_y; e_psi; vy; r];

    A_c = [0,            vx,                            1,                          0;
           0,             0,                            0,                          1;
           0,             0, -(Cf + Cr) / (M * vx), (-lf * Cf + lr * Cr) / (M * vx) - vx;
           0,             0, (-lf * Cf + lr * Cr) / (Iz * vx), -(lf^2 * Cf + lr^2 * Cr) / (Iz * vx)];

    B_c = [0;
           0;
           Cf / M;
           lf * Cf / Iz];

    % ----------------------------------------------------------------
    % EXACT DISCRETIZATION using matrix exponential (ZOH)
    % ----------------------------------------------------------------
    nx = 4;
    nu = 1;
    N = p.N;

    % Discretize A and B using augmented matrix with unit curvature
    % so we can scale g per step for curvature preview
    g_c_unit = [0; -vx; 0; 0];   % g_c per unit curvature

    M_aug = zeros(nx + nu + 1, nx + nu + 1);
    M_aug(1:nx, 1:nx) = A_c;
    M_aug(1:nx, nx+1) = B_c;
    M_aug(1:nx, nx+2) = g_c_unit;

    E = expm(M_aug * dt);
    A = E(1:nx, 1:nx);
    B = E(1:nx, nx+1);
    g_unit = E(1:nx, nx+2);   % g per unit curvature — scale by kappa
    % ----------------------------------------------------------------

    % ----------------------------------------------------------------
    % CURVATURE PREVIEW along the prediction horizon
    % Estimate where the vehicle will be at each future step and
    % look up the path curvature there.
    % ----------------------------------------------------------------
    n_ref = numel(ref.kappa);

    % Estimate local path spacing from reference points near current index
    i_lo = max(1, idx - 1);
    i_hi = min(n_ref, idx + 1);
    ds_local = hypot(ref.x(i_hi) - ref.x(i_lo), ref.y(i_hi) - ref.y(i_lo)) / (i_hi - i_lo);
    ds_local = max(ds_local, 0.01);   % guard against degenerate spacing

    % Indices per prediction step
    idx_per_step = max(1, round(vx * dt / ds_local));

    % Build curvature vector for the full horizon (including delay steps)
    n_delay = 0;
    if ~isempty(steer_buffer) && numel(steer_buffer) > 1
        n_delay = numel(steer_buffer) - 1;
    end

    kappa_horizon = zeros(n_delay + N, 1);
    for k = 1:(n_delay + N)
        future_idx = min(idx + k * idx_per_step, n_ref);
        kappa_horizon(k) = ref.kappa(future_idx);
    end
    % ----------------------------------------------------------------

    % ----------------------------------------------------------------
    % DELAY COMPENSATION with curvature preview
    % ----------------------------------------------------------------
    if n_delay > 0
        for d = 1:n_delay
            u_pending = steer_buffer(d);
            g_d = g_unit * kappa_horizon(d);
            x0 = A * x0 + B * u_pending + g_d;
        end
        delta_prev = steer_buffer(n_delay);
    end
    % ----------------------------------------------------------------

    % Curvature for the prediction horizon (after delay)
    kappa_pred = kappa_horizon(n_delay+1 : n_delay+N);

    Qbar = kron(eye(N), p.Q);
    Rbar = kron(eye(N), p.R);

    Sx = zeros(nx * N, nx);
    Su = zeros(nx * N, nu * N);
    Sg = zeros(nx * N, 1);
    A_power = eye(nx);
    for i = 1:N
        A_power = A_power * A;
        row = (i-1)*nx+1:i*nx;
        Sx(row, :) = A_power;
        g_sum = zeros(nx, 1);
        for j = 1:i
            A_ij = A^(i-j);
            Su(row, (j-1)*nu+1:j*nu) = A_ij * B;
            % Time-varying curvature disturbance
            g_j = g_unit * kappa_pred(j);
            g_sum = g_sum + A_ij * g_j;
        end
        Sg(row, :) = g_sum;
    end

    D = eye(N);
    D = D - [zeros(1, N); eye(N-1, N)];
    d0 = zeros(N, 1);
    d0(1) = delta_prev;

    x_free = Sx * x0 + Sg;

    H = Su' * Qbar * Su + Rbar + p.Rd * (D' * D);
    H = 0.5 * (H + H');   % enforce symmetry for quadprog

    %cost function
    f = Su' * Qbar * x_free - p.Rd * (D' * d0);

    % kappa_ff_gain = 0: curvature fully handled inside QP via g
    delta_ff = p.kappa_ff_gain * atan(L * kappa0);
    lb = -p.max_steer * ones(N, 1) - delta_ff;
    ub =  p.max_steer * ones(N, 1) - delta_ff;

    try
        opts = optimoptions('quadprog', 'Display', 'off');
        U = quadprog(2 * H, 2 * f, [], [], [], [], lb, ub, [], opts);

        if isempty(U)
            delta_corr = 0;
        else
            delta_corr = U(1);
        end
    catch
        delta_corr = -0.8 * e_y - 1.2 * e_psi - 0.05 * vy - 0.08 * r;
    end

    delta = delta_corr + delta_ff;
end

function kappa0 = interpolate_projected_curvature(kappa_ref, idx, seg_idx, seg_t)
    n = numel(kappa_ref);
    if ~isnan(seg_idx) && seg_idx >= 1 && seg_idx < n
        kappa0 = (1 - seg_t) * kappa_ref(seg_idx) + seg_t * kappa_ref(seg_idx + 1);
    else
        kappa0 = kappa_ref(idx);
    end
end


function vx = get_vx(state)
    if isfield(state, 'vx')
        vx = state.vx;
    else
        vx = state.v;
    end
end