function [a_des, lon] = LQR_controller(v_ref, v, lon, dt)
    % Longitudinal kinematic model:
    %   v(k+1) = v(k) + a_des(k) * dt
    %
    % Augmented error-state model for tracking:
    %   x = [e_v; z]
    %   e_v = v_ref - v
    %   z   = integral(e_v)
    %
    % Discrete dynamics:
    %   x(k+1) = A x(k) + B u(k)
    % with
    %   A = [1  0;
    %        dt 1]
    %   B = [-dt;
    %         0]
    % and control input u = a_des.

    ev = v_ref - v;
    lon.int_error = lon.int_error + ev * dt;

    x = [ev; lon.int_error];
    A = [1.0, 0.0;
         dt, 1.0];
    B = [-dt;
          0.0];

    K = solve_dlqr_gain(A, B, lon.Q, lon.R);
    a_des = -K * x;
    a_des = min(max(a_des, lon.a_min), lon.a_max);
end

function K = solve_dlqr_gain(A, B, Q, R)
    P = Q;

    for k = 1:200
        BtPB = B' * P * B;
        G = R + BtPB;
        P_next = A' * P * A - (A' * P * B) * (G \ (B' * P * A)) + Q;

        if norm(P_next - P, 'fro') < 1e-9
            P = P_next;
            break;
        end

        P = P_next;
    end

    K = (R + B' * P * B) \ (B' * P * A);
end
