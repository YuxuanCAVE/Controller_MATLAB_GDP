function cfg = default_config()
    cfg.controller.lateral = "mpc_combined";  % "stanley" | "pure_pursuit" | "mpc" | "mpc_combined"
    cfg.controller.longitudinal = "lqr";      % "pid" | "lqr" (ignored when lateral = "mpc_combined")

    cfg.sim.dt = 0.05;
    cfg.sim.T_end = 150;
    cfg.sim.max_travel_time = cfg.sim.T_end;
    cfg.sim.progress_window = 80;

    cfg.ref.path_file = fullfile('data', 'path_ref.mat');

    cfg.vehicle.accel_map_file = fullfile('data', 'Acc_mapData_noSlope.mat');
    cfg.vehicle.brake_map_file = fullfile('data', 'brake_mapData_noSlope.mat');
    cfg.vehicle.max_steer = deg2rad(35);       % max steering angle (rad)
    cfg.vehicle.max_steer_rate = deg2rad(40);   % max steering rate (rad/s)
    cfg.vehicle.delay.steer_s = 0.1;            % steering actuator delay (s)
    cfg.vehicle.delay.longitudinal_s = 0.1;     % throttle/brake actuator delay (s)

    % ── Speed reference ───────────────────────────────────────────────
    % To use constant speed: set mode = "constant" and constant_value
    % To use speed profile:  set mode = "profile" and profile_file
    cfg.speed.mode = "constant";   % "constant" | "profile"
    cfg.speed.constant_value = 10;  % used when mode = "constant" (m/s)
    cfg.speed.profile_file = fullfile( ...
        'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_5.mat');
    % Available profiles: peak_velocity_3, 4, 5, 7 .mat

    % ── Acceleration limits from actuator maps ────────────────────────
    % Computed from Acc_mapData_noSlope.mat and brake_mapData_noSlope.mat
    % at 60% pedal cap (max_pedal_publish = 0.6)
    %   a_max = max(Force_full) / M = 3.372 m/s^2
    %   a_min = -max(brake Force_full) / M = -7.357 m/s^2
    cfg.accel_limits.a_max = 3.372;
    cfg.accel_limits.a_min = -7.357;

    % ── Stanley lateral ───────────────────────────────────────────────
    cfg.stanley.k_cte = 1.8;
    cfg.stanley.k_soft = 2.0;
    cfg.stanley.k_yaw = 1.4;
    cfg.stanley.delta_ff_gain = 1.0;

    % ── Pure Pursuit lateral ──────────────────────────────────────────
    cfg.pure_pursuit.Ld_min = 4.0;
    cfg.pure_pursuit.Ld_gain = 0.70;
    cfg.pure_pursuit.Ld_max = 16.0;
    cfg.pure_pursuit.k_pp = 1.0;
    cfg.pure_pursuit.delta_ff_gain = 0.8;

    % ── MPC lateral only ──────────────────────────────────────────────
    cfg.mpc.N = 25;
    cfg.mpc.Q = diag([15, 12, 8, 10]);
    cfg.mpc.R = 5;
    cfg.mpc.Rd = 15.0;
    cfg.mpc.kappa_ff_gain = 0.5;
    cfg.mpc.max_steer = cfg.vehicle.max_steer;

    % ── Combined MPC (lateral + longitudinal) ─────────────────────────
    % Runs at 10 Hz (drive-by-wire limit)
    % State: [ey, epsi, vy, r, ev, z_lon]  (6 states)
    % Input: [delta, a_des]                 (2 inputs)
    cfg.mpc_combined.N = 25;
    cfg.mpc_combined.dt_ctrl = 0.10;     % 10 Hz controller rate
    cfg.mpc_combined.Q = diag([15, 12, 8, 10, 8, 20]);
    cfg.mpc_combined.R = diag([5, 0.5]);
    cfg.mpc_combined.Rd = diag([15, 2.0]);
    cfg.mpc_combined.kappa_ff_gain = 0.5;
    cfg.mpc_combined.max_steer = cfg.vehicle.max_steer;
    cfg.mpc_combined.a_min = cfg.accel_limits.a_min;
    cfg.mpc_combined.a_max = cfg.accel_limits.a_max;

    % ── PID longitudinal ──────────────────────────────────────────────
    cfg.lon_pid.kp = 2.5;
    cfg.lon_pid.ki = 2.0;
    cfg.lon_pid.kd = 0.08;
    cfg.lon_pid.a_min = cfg.accel_limits.a_min;
    cfg.lon_pid.a_max = cfg.accel_limits.a_max;
    cfg.lon_pid.i_term = 0;
    cfg.lon_pid.prev_ev = 0;

    % ── LQR longitudinal ─────────────────────────────────────────────
    cfg.lon_lqr.Q = diag([20, 10]);
    cfg.lon_lqr.R = 0.4;
    cfg.lon_lqr.a_min = cfg.accel_limits.a_min;
    cfg.lon_lqr.a_max = cfg.accel_limits.a_max;
    cfg.lon_lqr.int_error = 0.0;

    % ── Run settings ──────────────────────────────────────────────────
    cfg.run.root_dir = 'run';
    cfg.run.compare_longitudinal = false;  % true to compare PID vs LQR
    cfg.run.longitudinal_compare_set = ["pid", "lqr"];
end