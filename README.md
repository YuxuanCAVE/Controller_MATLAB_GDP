# ControllerMatlab

MATLAB closed-loop path-following framework with:

- lateral controllers: `mpc`, `stanley`, `pure_pursuit`
- longitudinal controllers: `pid`, `lqr`
- nonlinear vehicle plant for simulation
- configurable speed reference, actuator delay, and comparison runs

The entry point is [main.m](/home/yuxuan/ControllerMatlab/main.m).

## Quick Start

Open MATLAB in the project root and run:

```matlab
main
```

All outputs are saved under `run/`.

## Main Workflow

The current workflow is:

```text
main
-> load config
-> load reference path
-> load reference speed
-> load vehicle parameters
-> run closed-loop simulation
-> save plots / result / summary
```

Core files:

- [main.m](/home/yuxuan/ControllerMatlab/main.m): top-level entry
- [config/default_config.m](/home/yuxuan/ControllerMatlab/config/default_config.m): all main settings
- [simulation/run_closed_loop.m](/home/yuxuan/ControllerMatlab/simulation/run_closed_loop.m): closed-loop loop
- [plotting/save_run_plots.m](/home/yuxuan/ControllerMatlab/plotting/save_run_plots.m): per-run plots
- [plotting/save_longitudinal_comparison.m](/home/yuxuan/ControllerMatlab/plotting/save_longitudinal_comparison.m): PID/LQR comparison plots

## What To Configure

Most usage is controlled from [default_config.m](/home/yuxuan/ControllerMatlab/config/default_config.m).

### Controller Selection

```matlab
cfg.controller.lateral = "mpc";         % "stanley" | "pure_pursuit" | "mpc"
cfg.controller.longitudinal = "lqr";    % "pid" | "lqr"
```

- `lateral`: selects the steering controller
- `longitudinal`: selects the speed controller

## Simulation Settings

```matlab
cfg.sim.dt = 0.05;
cfg.sim.T_end = 40;
cfg.sim.progress_window = 80;
```

- `dt`: simulation sample time
- `T_end`: maximum simulation duration
- `progress_window`: local search window for the nearest reference point

## Reference Path and Speed

```matlab
cfg.ref.path_file = fullfile('data', 'path_ref.mat');

cfg.speed.mode = "profile";   % "constant" | "profile"
cfg.speed.constant_value = 9.0;
cfg.speed.profile_file = fullfile( ...
    'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_5.mat');
```

- `path_file`: XY reference path
- `speed.mode = "constant"`: use one fixed speed along the path
- `speed.mode = "profile"`: use a speed profile from a MAT file
- `speed.profile_file`: file containing `pathv_ref`

Current speed-profile logic:

- reads `pathv_ref`
- uses samples `26:384`
- matches the current path length of `359`

## Vehicle and Actuator Settings

```matlab
cfg.vehicle.max_steer = deg2rad(35);
cfg.vehicle.max_steer_rate = deg2rad(40);
cfg.vehicle.delay.steer_s = 0.1;
cfg.vehicle.delay.longitudinal_s = 0.1;
```

- `max_steer`: steering angle limit
- `max_steer_rate`: steering rate limit
- `delay.steer_s`: delay between lateral controller output and executed steering
- `delay.longitudinal_s`: delay between longitudinal controller output and executed acceleration

## Lateral Controller Parameters

### Stanley

```matlab
cfg.stanley.k_cte
cfg.stanley.k_soft
cfg.stanley.k_yaw
cfg.stanley.delta_ff_gain
```

- cross-track gain
- low-speed softening term
- heading-error gain
- curvature feedforward gain

### Pure Pursuit

```matlab
cfg.pure_pursuit.Ld_min
cfg.pure_pursuit.Ld_gain
cfg.pure_pursuit.Ld_max
cfg.pure_pursuit.k_pp
cfg.pure_pursuit.delta_ff_gain
```

- minimum / gain / maximum look-ahead distance
- steering gain
- curvature feedforward gain

### MPC

```matlab
cfg.mpc.N
cfg.mpc.Q
cfg.mpc.R
cfg.mpc.Rd
cfg.mpc.kappa_ff_gain
cfg.mpc.max_steer
```

- `N`: prediction horizon
- `Q`: state-error weight matrix
- `R`: steering-effort penalty
- `Rd`: steering-rate-change penalty
- `kappa_ff_gain`: curvature feedforward gain
- `max_steer`: steering constraint used by MPC

Current MPC type:

- linear MPC
- based on a linearized dynamic bicycle error model
- solved as a QP

## Longitudinal Controller Parameters

### PID

```matlab
cfg.lon_pid.kp
cfg.lon_pid.ki
cfg.lon_pid.kd
cfg.lon_pid.a_min
cfg.lon_pid.a_max
```

- PID gains on speed error
- desired acceleration limits

### LQR

```matlab
cfg.lon_lqr.Q
cfg.lon_lqr.R
cfg.lon_lqr.a_min
cfg.lon_lqr.a_max
```

- `Q`: weight on speed error and integrated speed error
- `R`: penalty on desired acceleration effort
- `a_min`, `a_max`: desired acceleration bounds

Current LQR model:

- longitudinal kinematic model
- `v(k+1) = v(k) + a_des(k) * dt`

## Run Modes

### Single Run

```matlab
cfg.run.compare_longitudinal = false;
```

Runs one lateral controller and one longitudinal controller.

Typical output folder:

```text
run/<timestamp>_<lateral>_<longitudinal>/
```

### Longitudinal Comparison Run

```matlab
cfg.run.compare_longitudinal = true;
cfg.run.longitudinal_compare_set = ["pid", "lqr"];
```

Runs the same lateral controller with multiple longitudinal controllers.

Typical output folder:

```text
run/<timestamp>_<lateral>_lon_compare/
  pid/
  lqr/
  comparison_summary.txt
  longitudinal_comparison.png
```

## Output Files

Per single run or per case folder, typical files are:

- `path_tracking.png`
- `tracking_errors.png`
- `longitudinal.png`
- `lateral_dynamics.png`
- `result.mat`
- `summary.txt`

Comparison mode also adds:

- `comparison.mat`
- `comparison_summary.txt`
- `longitudinal_comparison.png`

## Current Model Notes

- Reference point selection uses [nearest_path_ref_point.m](/home/yuxuan/ControllerMatlab/utils/nearest_path_ref_point.m)
- Longitudinal plant uses force balance plus pedal maps
- Lateral simulation plant is nonlinear
- MPC is linearized, not nonlinear MPC

## Minimal Typical Usage

### Run MPC + LQR with speed profile

```matlab
cfg.controller.lateral = "mpc";
cfg.controller.longitudinal = "lqr";
cfg.speed.mode = "profile";
cfg.run.compare_longitudinal = false;
```

### Compare PID vs LQR under the same lateral controller

```matlab
cfg.controller.lateral = "mpc";
cfg.run.compare_longitudinal = true;
cfg.run.longitudinal_compare_set = ["pid", "lqr"];
```
