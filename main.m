clear;
clc;
close all;

project_root = fileparts(which('main'));
if isempty(project_root)
    project_root = pwd;
end

addpath(fullfile(project_root, 'config'));
addpath(fullfile(project_root, 'reference'));
addpath(fullfile(project_root, 'controllers', 'lateral'));
addpath(fullfile(project_root, 'controllers', 'longitudinal'));
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'simulation'));
addpath(fullfile(project_root, 'plotting'));
addpath(fullfile(project_root, 'utils'));

cfg = default_config();
ref = load_reference_path(cfg.ref.path_file);
ref = load_reference_speed(ref, cfg.speed);
veh = load_vehicle_params(cfg.vehicle.accel_map_file, cfg.vehicle.brake_map_file);
veh.max_steer = cfg.vehicle.max_steer;
veh.max_steer_rate = cfg.vehicle.max_steer_rate;

stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
if cfg.run.compare_longitudinal
    run_name = sprintf('%s_%s_lon_compare', stamp, char(cfg.controller.lateral));
    run_dir = fullfile(project_root, cfg.run.root_dir, run_name);
    if ~exist(run_dir, 'dir')
        mkdir(run_dir);
    end

    comparison.cfg = cfg;
    comparison.results = struct();
    controllers = cellstr(cfg.run.longitudinal_compare_set);

    for i = 1:numel(controllers)
        cfg_case = cfg;
        cfg_case.controller.longitudinal = string(controllers{i});
        case_dir = fullfile(run_dir, controllers{i});
        if ~exist(case_dir, 'dir')
            mkdir(case_dir);
        end

        result = run_closed_loop(cfg_case, ref, veh);
        save_run_plots(cfg_case, ref, result, case_dir);
        save(fullfile(case_dir, 'result.mat'), 'cfg_case', 'ref', 'veh', 'result');
        write_single_summary(result, case_dir);
        comparison.results.(controllers{i}) = result;
    end

    save(fullfile(run_dir, 'comparison.mat'), 'comparison', 'ref', 'veh');
    save_longitudinal_comparison(cfg, ref, comparison, run_dir);
    fprintf('Saved comparison outputs to: %s\n', run_dir);
else
    run_name = sprintf('%s_%s_%s', stamp, ...
        char(cfg.controller.lateral), char(cfg.controller.longitudinal));
    run_dir = fullfile(project_root, cfg.run.root_dir, run_name);
    if ~exist(run_dir, 'dir')
        mkdir(run_dir);
    end

    result = run_closed_loop(cfg, ref, veh);
    save_run_plots(cfg, ref, result, run_dir);

    save(fullfile(run_dir, 'result.mat'), 'cfg', 'ref', 'veh', 'result');
    write_single_summary(result, run_dir);

    fprintf('Controller: %s\n', char(result.metrics.controller));
    fprintf('RMS cross-track error  : %.3f m\n', result.metrics.rms_cte);
    fprintf('Peak cross-track error : %.3f m\n', result.metrics.peak_cte);
    fprintf('RMS heading error      : %.3f deg\n', result.metrics.rms_epsi_deg);
    fprintf('Saved run outputs to   : %s\n', run_dir);
end

function write_single_summary(result, run_dir)
fid = fopen(fullfile(run_dir, 'summary.txt'), 'w');
fprintf(fid, 'Controller: %s\n', char(result.metrics.controller));
fprintf(fid, 'RMS cross-track error  : %.3f m\n', result.metrics.rms_cte);
fprintf(fid, 'Peak cross-track error : %.3f m\n', result.metrics.peak_cte);
fprintf(fid, 'RMS heading error      : %.3f deg\n', result.metrics.rms_epsi_deg);
fprintf(fid, 'RMS speed error        : %.3f m/s\n', result.metrics.rms_speed_error);
fprintf(fid, 'Peak speed error       : %.3f m/s\n', result.metrics.peak_speed_error);
fprintf(fid, 'Final speed            : %.3f m/s\n', result.metrics.final_speed);
fprintf(fid, 'Run directory          : %s\n', run_dir);
fclose(fid);
end
