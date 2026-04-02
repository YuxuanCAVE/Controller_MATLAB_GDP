function result = run_closed_loop(cfg, ref, veh)
    dt = cfg.sim.dt;
    Nsim = round(cfg.sim.T_end / dt);
    steer_delay_steps = max(round(cfg.vehicle.delay.steer_s / dt), 0);
    lon_delay_steps = max(round(cfg.vehicle.delay.longitudinal_s / dt), 0);

    state.x = ref.x(1);
    state.y = ref.y(1);
    state.yaw = ref.yaw(1);
    state.v = 0.5;
    state.vx = state.v;
    state.vy = 0.0;
    state.r = 0.0;
    state.beta = 0.0;
    state.delta = 0.0;

    switch cfg.controller.longitudinal
        case "pid"
            lon = cfg.lon_pid;
        case "lqr"
            lon = cfg.lon_lqr;
        otherwise
            error('Unknown longitudinal controller: %s', cfg.controller.longitudinal);
    end

    idx_progress = 1;
    steer_delay_buffer = state.delta * ones(steer_delay_steps + 1, 1);
    lon_delay_buffer = zeros(lon_delay_steps + 1, 1);

    log.t = zeros(Nsim, 1);
    log.x = zeros(Nsim, 1);
    log.y = zeros(Nsim, 1);
    log.yaw = zeros(Nsim, 1);
    log.v = zeros(Nsim, 1);
    log.v_ref = zeros(Nsim, 1);
    log.delta = zeros(Nsim, 1);
    log.delta_cmd_raw = zeros(Nsim, 1);
    log.delta_cmd_exec = zeros(Nsim, 1);
    log.ax = zeros(Nsim, 1);
    log.ax_cmd = zeros(Nsim, 1);
    log.ax_cmd_exec = zeros(Nsim, 1);
    log.throttle = zeros(Nsim, 1);
    log.brake = zeros(Nsim, 1);
    log.vy = zeros(Nsim, 1);
    log.r = zeros(Nsim, 1);
    log.beta = zeros(Nsim, 1);
    log.ay = zeros(Nsim, 1);
    log.alpha_f = zeros(Nsim, 1);
    log.alpha_r = zeros(Nsim, 1);
    log.Fy_f = zeros(Nsim, 1);
    log.Fy_r = zeros(Nsim, 1);
    log.F_resist = zeros(Nsim, 1);
    log.F_required = zeros(Nsim, 1);
    log.F_total = zeros(Nsim, 1);
    log.F_drive = zeros(Nsim, 1);
    log.idx = zeros(Nsim, 1);
    log.e_ct = zeros(Nsim, 1);
    log.e_psi = zeros(Nsim, 1);

    stop_idx = Nsim;

    for k = 1:Nsim
        [idx_near, e_ct, e_psi] = track_errors( ...
            state.x, state.y, state.yaw, ref, idx_progress, cfg.sim.progress_window);
        idx_progress = max(idx_progress, idx_near);
        v_ref_now = ref.v_ref(max(min(idx_near, numel(ref.v_ref)), 1));

        switch cfg.controller.lateral
            case "stanley"
                delta_cmd = stanley_lateral( ...
                    state.x, state.y, state.yaw, state.v, ref, veh.L, cfg.stanley, ...
                    idx_progress, cfg.sim.progress_window);

            case "pure_pursuit"
                delta_cmd = pure_pursuit_lateral( ...
                    state.x, state.y, state.yaw, state.v, ref, veh.L, cfg.pure_pursuit, ...
                    idx_progress, cfg.sim.progress_window);

            case "mpc"
                delta_cmd = mpc_lateral(state, ref, veh, dt, cfg.mpc, ...
                    idx_progress, cfg.sim.progress_window, steer_delay_buffer);

            otherwise
                error('Unknown lateral controller: %s', cfg.controller.lateral);
        end

        delta_cmd_raw = max(min(delta_cmd, veh.max_steer), -veh.max_steer);
        [delta_cmd_delayed, steer_delay_buffer] = apply_delay(delta_cmd_raw, steer_delay_buffer);
        delta_cmd_exec = rate_limit(delta_cmd_delayed, state.delta, veh.max_steer_rate, dt);

        switch cfg.controller.longitudinal
            case "pid"
                [a_des_raw, lon] = PID_controller(v_ref_now, state.v, lon, dt);
            case "lqr"
                [a_des_raw, lon] = LQR_controller(v_ref_now, state.v, lon, dt);
            otherwise
                error('Unknown longitudinal controller: %s', cfg.controller.longitudinal);
        end

        [a_des_exec, lon_delay_buffer] = apply_delay(a_des_raw, lon_delay_buffer);
        lon_model = longitudinal_model(state.v, a_des_exec, veh);

        [state, ax_actual, lat] = lateral_model(state, delta_cmd_exec, lon_model, dt, veh);

        log.t(k) = (k - 1) * dt;
        log.x(k) = state.x;
        log.y(k) = state.y;
        log.yaw(k) = state.yaw;
        log.v(k) = state.v;
        log.v_ref(k) = v_ref_now;
        log.vy(k) = state.vy;
        log.r(k) = state.r;
        log.beta(k) = state.beta;
        log.delta(k) = state.delta;
        log.delta_cmd_raw(k) = delta_cmd_raw;
        log.delta_cmd_exec(k) = delta_cmd_exec;
        log.ax(k) = ax_actual;
        log.ax_cmd(k) = a_des_raw;
        log.ax_cmd_exec(k) = a_des_exec;
        log.throttle(k) = lon_model.throttle_pct;
        log.brake(k) = lon_model.brake_pct;
        log.ay(k) = lat.ay;
        log.alpha_f(k) = lat.alpha_f;
        log.alpha_r(k) = lat.alpha_r;
        log.Fy_f(k) = lat.Fy_f;
        log.Fy_r(k) = lat.Fy_r;
        log.F_resist(k) = lon_model.F_resist;
        log.F_required(k) = lon_model.F_required;
        log.F_total(k) = lon_model.F_total;
        log.F_drive(k) = lon_model.F_drive_actual;
        log.idx(k) = idx_near;
        log.e_ct(k) = e_ct;
        log.e_psi(k) = e_psi;

        if idx_progress >= numel(ref.x) - 2 && hypot(state.x - ref.x(end), state.y - ref.y(end)) < 1.5
            stop_idx = k;
            break;
        end
    end

    fields = fieldnames(log);
    for ii = 1:numel(fields)
        log.(fields{ii}) = log.(fields{ii})(1:stop_idx);
    end

    result.log = log;
    result.final_state = state;
    result.metrics.rms_cte = sqrt(mean(log.e_ct.^2));
    result.metrics.rms_epsi_deg = sqrt(mean(rad2deg(log.e_psi).^2));
    result.metrics.peak_cte = max(abs(log.e_ct));
    speed_error = log.v_ref - log.v;
    result.metrics.rms_speed_error = sqrt(mean(speed_error.^2));
    result.metrics.peak_speed_error = max(abs(speed_error));
    result.metrics.final_speed = log.v(end);
    result.metrics.lateral_controller = cfg.controller.lateral;
    result.metrics.longitudinal_controller = cfg.controller.longitudinal;
    result.metrics.controller = sprintf('%s + %s', ...
        char(cfg.controller.lateral), char(cfg.controller.longitudinal));
end

function [u_exec, buffer] = apply_delay(u_cmd, buffer)
    if numel(buffer) <= 1
        u_exec = u_cmd;
        buffer(1) = u_cmd;
        return;
    end

    u_exec = buffer(1);
    buffer(1:end-1) = buffer(2:end);
    buffer(end) = u_cmd;
end