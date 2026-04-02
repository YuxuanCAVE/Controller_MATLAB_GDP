function [state_next, dbg] = coupled_bicycle_dynamics(state, delta_cmd, lon_force, lat_force, veh, dt)
    % Coupled 3DOF planar bicycle dynamics.
    % This module combines longitudinal and lateral forces into state updates.

    vx = max(get_vx(state), 0.5);
    vy = state.vy;
    r = state.r;
    yaw = state.yaw;

    M = veh.M;
    Iz = veh.Iz;
    lf = veh.lf;
    lr = veh.lr;

    Fx = lon_force.F_total;
    Fy_f = lat_force.Fy_f;
    Fy_r = lat_force.Fy_r;

    vx_dot = (Fx - Fy_f * sin(delta_cmd)) / M + vy * r;
    vy_dot = (Fy_f * cos(delta_cmd) + Fy_r) / M - vx * r;
    r_dot = (lf * Fy_f * cos(delta_cmd) - lr * Fy_r) / Iz;

    vx_next = max(0, vx + vx_dot * dt);
    vy_next = vy + vy_dot * dt;
    r_next = r + r_dot * dt;
    yaw_next = angle_wrap(yaw + r_next * dt);

    x_dot = vx * cos(yaw) - vy * sin(yaw);
    y_dot = vx * sin(yaw) + vy * cos(yaw);
    x_next = state.x + x_dot * dt;
    y_next = state.y + y_dot * dt;

    beta_next = atan2(vy_next, max(vx_next, 0.5));
    ay = vy_dot + vx * r;

    state_next = state;
    state_next.x = x_next;
    state_next.y = y_next;
    state_next.yaw = yaw_next;
    state_next.v = vx_next;
    state_next.vx = vx_next;
    state_next.vy = vy_next;
    state_next.r = r_next;
    state_next.beta = beta_next;
    state_next.delta = delta_cmd;

    dbg.ax = vx_dot;
    dbg.ay = ay;
    dbg.Fx = Fx;
    dbg.Fy_f = Fy_f;
    dbg.Fy_r = Fy_r;
    dbg.alpha_f = lat_force.alpha_f;
    dbg.alpha_r = lat_force.alpha_r;
end

function vx = get_vx(state)
    if isfield(state, 'vx')
        vx = state.vx;
    else
        vx = state.v;
    end
end
