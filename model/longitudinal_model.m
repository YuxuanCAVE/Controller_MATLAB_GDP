function lon_force = longitudinal_model(vx, a_des, veh)
    % Main longitudinal model wrapper used by the current project.
    % Internally it delegates the force generation to a standalone module.
    Velocity = vx;

    A = veh.A;
    B = veh.B;
    C = veh.C;
    M = veh.M;

    F_grad = 0;
    F_aero_plus_road = A + B * Velocity + C * Velocity^2;
    F_resist = F_aero_plus_road + F_grad;

    throttle_pct = 0;
    brake_pct = 0;
    ACC_req = 0;
    BRK_req = 0;
    F_drive_actual = 0;

    if a_des >= 0
        F_required = M * a_des + F_resist;
        F_tractive_required = F_required;

        ACC_req = interp1( ...
            veh.acc.force_full, ...
            veh.acc.acc_full, ...
            F_tractive_required, ...
            'linear', 'extrap');

        throttle_pct = (ACC_req / max(veh.acc.acc_full)) * veh.max_pedal_publish;
        throttle_pct = min(max(throttle_pct, 0), veh.max_pedal_publish);

        ACC_internal = (throttle_pct / veh.max_pedal_publish) * max(veh.acc.acc_full);
        F_drive_actual = interp1( ...
            veh.acc.acc_full, ...
            veh.acc.force_full, ...
            ACC_internal, ...
            'linear', 'extrap');
    else
        F_required = M * a_des + F_resist;
        F_brake_required = max(-F_required, 0);

        BRK_req = interp1( ...
            veh.brk.force_full, ...
            veh.brk.brake_full, ...
            F_brake_required, ...
            'linear', 'extrap');

        brake_pct = (BRK_req / max(veh.brk.brake_full)) * veh.max_pedal_publish;
        brake_pct = min(max(brake_pct, 0), veh.max_pedal_publish);

        BRK_internal = (brake_pct / veh.max_pedal_publish) * max(veh.brk.brake_full);
        F_brake_actual = interp1( ...
            veh.brk.brake_full, ...
            veh.brk.force_full, ...
            BRK_internal, ...
            'linear', 'extrap');

        F_drive_actual = -F_brake_actual;
    end

    F_total = F_drive_actual - F_resist;
    a_actual = F_total / M;

    lon_force.a_des = a_des;
    lon_force.a_actual = a_actual;
    lon_force.throttle_pct = throttle_pct;
    lon_force.brake_pct = brake_pct;
    lon_force.ACC_req = ACC_req;
    lon_force.BRK_req = BRK_req;
    lon_force.F_grad = F_grad;
    lon_force.F_aero_plus_road = F_aero_plus_road;
    lon_force.F_resist = F_resist;
    lon_force.F_required = F_required;
    lon_force.F_drive_actual = F_drive_actual;
    lon_force.F_total = F_total;
end
