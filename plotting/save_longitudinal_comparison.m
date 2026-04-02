function save_longitudinal_comparison(cfg, ref, comparison, run_dir)
    fig_visible = 'off';
    keep_open = false;
    if usejava('desktop')
        fig_visible = 'on';
        keep_open = true;
    end

    controllers = fieldnames(comparison.results);
    colors = lines(numel(controllers));

    fig = figure('Color', 'w', 'Name', 'Longitudinal Controller Comparison', ...
        'Visible', fig_visible);

    subplot(2, 2, 1);
    plot(ref.x, ref.y, 'k--', 'LineWidth', 1.2);
    hold on;
    for i = 1:numel(controllers)
        name = controllers{i};
        log = comparison.results.(name).log;
        plot(log.x, log.y, 'LineWidth', 1.6, 'Color', colors(i, :));
    end
    axis equal;
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    title(sprintf('Path Tracking Comparison (%s)', char(cfg.controller.lateral)));
    legend_entries = [{'Reference'}, controllers'];
    legend(legend_entries, 'Location', 'best');

    subplot(2, 2, 2);
    for i = 1:numel(controllers)
        name = controllers{i};
        log = comparison.results.(name).log;
        plot(log.t, log.v, 'LineWidth', 1.5, 'Color', colors(i, :));
        hold on;
    end
    ref_log = comparison.results.(controllers{1}).log;
    plot(ref_log.t, ref_log.v_ref, '--k', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]');
    ylabel('v [m/s]');
    title('Speed Tracking');
    legend([controllers', {'Reference'}], 'Location', 'best');

    subplot(2, 2, 3);
    for i = 1:numel(controllers)
        name = controllers{i};
        log = comparison.results.(name).log;
        plot(log.t, log.ax_cmd_exec, '--', 'LineWidth', 1.1, 'Color', colors(i, :));
        hold on;
        plot(log.t, log.ax, 'LineWidth', 1.5, 'Color', colors(i, :));
    end
    grid on;
    xlabel('Time [s]');
    ylabel('a_x [m/s^2]');
    title('Executed vs Actual Longitudinal Acceleration');

    subplot(2, 2, 4);
    for i = 1:numel(controllers)
        name = controllers{i};
        log = comparison.results.(name).log;
        speed_error = log.v_ref - log.v;
        plot(log.t, speed_error, 'LineWidth', 1.5, 'Color', colors(i, :));
        hold on;
    end
    grid on;
    xlabel('Time [s]');
    ylabel('v_{ref} - v [m/s]');
    title('Speed Error');
    legend(controllers, 'Location', 'best');

    exportgraphics(fig, fullfile(run_dir, 'longitudinal_comparison.png'), 'Resolution', 150);
    savefig(fig, fullfile(run_dir, 'longitudinal_comparison.fig'));

    write_comparison_summary(cfg, comparison, run_dir);

    if ~keep_open
        close(fig);
    end
end

function write_comparison_summary(cfg, comparison, run_dir)
    controllers = fieldnames(comparison.results);
    fid = fopen(fullfile(run_dir, 'comparison_summary.txt'), 'w');

    fprintf(fid, 'Lateral controller      : %s\n', char(cfg.controller.lateral));
    fprintf(fid, 'Reference speed mode    : %s\n', char(cfg.speed.mode));
    if isfield(cfg.speed, 'profile_file') && cfg.speed.mode == "profile"
        fprintf(fid, 'Reference speed file    : %s\n', cfg.speed.profile_file);
    elseif isfield(cfg.speed, 'constant_value') && cfg.speed.mode == "constant"
        fprintf(fid, 'Reference speed value   : %.3f m/s\n', cfg.speed.constant_value);
    end
    fprintf(fid, 'Compared longitudinal controllers:\n');
    for i = 1:numel(controllers)
        fprintf(fid, '  - %s\n', controllers{i});
    end
    fprintf(fid, '\n');
    fprintf(fid, '%-12s %-12s %-12s %-12s %-12s %-12s\n', ...
        'Controller', 'RMS_CTE[m]', 'Peak_CTE[m]', 'RMS_Epsi[deg]', ...
        'RMS_Verr', 'Final_v');

    for i = 1:numel(controllers)
        name = controllers{i};
        metrics = comparison.results.(name).metrics;
        fprintf(fid, '%-12s %-12.3f %-12.3f %-12.3f %-12.3f %-12.3f\n', ...
            name, metrics.rms_cte, metrics.peak_cte, metrics.rms_epsi_deg, ...
            metrics.rms_speed_error, metrics.final_speed);
    end

    fclose(fid);
end
