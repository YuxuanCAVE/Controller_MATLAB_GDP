% Main lateral plant wrapper used by the project.
% This wrapper now delegates to:
%   1) lateral_tire_model
%   2) coupled_bicycle_dynamics
%
% Input:
%   state, delta_cmd, lon_model, dt, veh
%
% Output:
%   updated state, longitudinal acceleration ax, and lateral debug struct
function [state, ax, lat] = lateral_model(state, delta_cmd, lon_model, dt, veh)
    vx = get_vx(state);
    lat_force = lateral_tire_model(vx, state.vy, state.r, delta_cmd, veh);
    [state, dbg] = coupled_bicycle_dynamics(state, delta_cmd, lon_model, lat_force, veh, dt);

    ax = dbg.ax;
    lat.vx = get_vx(state);
    lat.vy = state.vy;
    lat.r = state.r;
    lat.beta = state.beta;
    lat.ay = dbg.ay;
    lat.alpha_f = dbg.alpha_f;
    lat.alpha_r = dbg.alpha_r;
    lat.Fy_f = dbg.Fy_f;
    lat.Fy_r = dbg.Fy_r;
end

function vx = get_vx(state)
    if isfield(state, 'vx')
        vx = state.vx;
    else
        vx = state.v;
    end
end
