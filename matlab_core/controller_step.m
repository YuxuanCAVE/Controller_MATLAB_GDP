function [command, memory, debug] = controller_step(meas, ref, memory)
    x = meas.x;
    y = meas.y;
    yaw = meas.yaw;
    vx = meas.vx;
    vy = meas.vy;
    r = meas.r;
    delta_prev = meas.delta;

    [idx, xr, yr, psi_ref, ey_proj, seg_idx, seg_t] = nearest_path_ref_point( ...
        x, y, ref.x, ref.y, idx_hint, window);

    kappa0 = interpolate_projected_curvature(ref.kappa, idx, seg_idx, seg_t);

    dx = x - xr;
    dy = y - yr;
    e_y = -sin(psi_ref) * dx + cos(psi_ref) * dy;
    e_psi = angle_wrap(yaw - psi_ref);

    Cf = 1.3;
    Cr = 1.3;
    M = 1948;
    Iz = 3712;
    lf = 1.214;
    
    L = 2.720;
    lr = L - lf;
    
    

    %lateral control 
    steering =

    %longitunal control
    throttle / brake =

    %command
    command = zeros(7,1);
    command(1) = 0;
    command(2) = 0;
    command(3) = 0;
    
    %4) debug
    debug = struct();
