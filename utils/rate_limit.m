function u = rate_limit(u_cmd, u_prev, u_rate_max, dt)
    du_max = u_rate_max * dt;
    du = u_cmd - u_prev;
    du = min(max(du, -du_max), du_max);
    u = u_prev + du;
end
