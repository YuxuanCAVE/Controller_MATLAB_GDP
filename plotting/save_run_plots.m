function save_run_plots(cfg, ref, result, run_dir)
    log = result.log;
    metrics = result.metrics;
    run_info = struct( ...
        'single_loop_time_s', metrics.single_loop_time_s, ...
        'max_travel_time_s', metrics.max_travel_time_s, ...
        'goal_reached', metrics.goal_reached, ...
        'timeout', metrics.timeout, ...
        'termination_reason', metrics.termination_reason, ...
        'controller', metrics.controller);
    fig_visible = 'off';
    keep_open = false;
    if usejava('desktop')
        fig_visible = 'on';
        keep_open = true;
    end

    fig1 = figure('Color', 'w', 'Name', 'Path Tracking', 'Visible', fig_visible);
    plot(ref.x, ref.y, 'k--', 'LineWidth', 1.4);
    hold on;
    plot(log.x, log.y, 'b', 'LineWidth', 2.0);
    plot(ref.x(1), ref.y(1), 'go', 'MarkerSize', 8, 'LineWidth', 2);
    plot(ref.x(end), ref.y(end), 'rs', 'MarkerSize', 8, 'LineWidth', 2);
    axis equal;
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    legend('Reference path', 'Vehicle trajectory', 'Start', 'End', 'Location', 'best');
    title(sprintf('Path Tracking using %s + %s | loop time = %.2f s | %s', ...
        char(cfg.controller.lateral), char(cfg.controller.longitudinal), ...
        metrics.single_loop_time_s, char(metrics.termination_reason)));

    fig2 = figure('Color', 'w', 'Name', 'Tracking Errors', 'Visible', fig_visible);
    subplot(3, 1, 1);
    plot(log.t, log.e_ct, 'LineWidth', 1.4);
    grid on;
    ylabel('e_{ct} [m]');
    title(sprintf('RMS CTE = %.3f m, Peak CTE = %.3f m', metrics.rms_cte, metrics.peak_cte));
    subplot(3, 1, 2);
    plot(log.t, rad2deg(log.e_psi), 'LineWidth', 1.4);
    grid on;
    ylabel('e_{\\psi} [deg]');
    subplot(3, 1, 3);
    plot(log.t, rad2deg(log.delta_cmd_raw), '--', 'LineWidth', 1.2);
    hold on;
    plot(log.t, rad2deg(log.delta_cmd_exec), 'LineWidth', 1.4);
    plot(log.t, rad2deg(log.delta), ':', 'LineWidth', 1.2);
    grid on;
    ylabel('\\delta [deg]');
    legend('Commanded', 'Executed', 'State', 'Location', 'best');
    xlabel('Time [s]');

    fig3 = figure('Color', 'w', 'Name', 'Longitudinal', 'Visible', fig_visible);
    subplot(4, 1, 1);
    plot(log.t, log.v, 'LineWidth', 1.4);
    hold on;
    plot(log.t, log.v_ref, '--k', 'LineWidth', 1.2);
    grid on;
    ylabel('v [m/s]');
    legend('Actual', 'Reference', 'Location', 'best');
    subplot(4, 1, 2);
    plot(log.t, log.throttle * 100, 'LineWidth', 1.4);
    hold on;
    plot(log.t, log.brake * 100, 'LineWidth', 1.4);
    grid on;
    ylabel('Pedal [%]');
    legend('Throttle', 'Brake', 'Location', 'best');
    subplot(4, 1, 3);
    plot(log.t, log.ax_cmd, '--', 'LineWidth', 1.2);
    hold on;
    plot(log.t, log.ax_cmd_exec, 'LineWidth', 1.4);
    plot(log.t, log.ax, 'LineWidth', 1.4);
    grid on;
    ylabel('a_x [m/s^2]');
    legend('Commanded', 'Executed', 'Actual', 'Location', 'best');
    subplot(4, 1, 4);
    plot(log.t, log.idx, 'LineWidth', 1.4);
    grid on;
    ylabel('Path idx');
    xlabel('Time [s]');

    fig4 = figure('Color', 'w', 'Name', 'Lateral Dynamics', 'Visible', fig_visible);
    subplot(4, 1, 1);
    plot(log.t, log.v, 'LineWidth', 1.4);
    hold on;
    plot(log.t, sqrt(log.v.^2 + log.vy.^2), '--', 'LineWidth', 1.2);
    grid on;
    ylabel('Speed [m/s]');
    legend('v_x', 'v', 'Location', 'best');
    title('Lateral Dynamic States');
    subplot(4, 1, 2);
    plot(log.t, rad2deg(log.beta), 'LineWidth', 1.4);
    hold on;
    plot(log.t, rad2deg(log.r), 'LineWidth', 1.4);
    grid on;
    ylabel('Angle / Rate');
    legend('\beta [deg]', 'r [deg/s]', 'Location', 'best');
    subplot(4, 1, 3);
    plot(log.t, log.ay, 'LineWidth', 1.4);
    hold on;
    plot(log.t, (log.v .^ 2) .* abs(interp1(1:numel(ref.kappa), abs(ref.kappa), max(log.idx, 1), 'nearest', 'extrap')), '--', 'LineWidth', 1.2);
    grid on;
    ylabel('a_y [m/s^2]');
    legend('Model a_y', 'v^2|\kappa|', 'Location', 'best');
    subplot(4, 1, 4);
    plot(log.t, rad2deg(log.alpha_f), 'LineWidth', 1.4);
    hold on;
    plot(log.t, rad2deg(log.alpha_r), 'LineWidth', 1.4);
    grid on;
    ylabel('\alpha [deg]');
    xlabel('Time [s]');
    legend('\alpha_f', '\alpha_r', 'Location', 'best');

    exportgraphics(fig1, fullfile(run_dir, 'path_tracking.png'), 'Resolution', 150);
    exportgraphics(fig2, fullfile(run_dir, 'tracking_errors.png'), 'Resolution', 150);
    exportgraphics(fig3, fullfile(run_dir, 'longitudinal.png'), 'Resolution', 150);
    exportgraphics(fig4, fullfile(run_dir, 'lateral_dynamics.png'), 'Resolution', 150);

    savefig(fig1, fullfile(run_dir, 'path_tracking.fig'));
    savefig(fig2, fullfile(run_dir, 'tracking_errors.fig'));
    savefig(fig3, fullfile(run_dir, 'longitudinal.fig'));
    savefig(fig4, fullfile(run_dir, 'lateral_dynamics.fig'));

    save(fullfile(run_dir, 'cfg.mat'), 'cfg');
    save(fullfile(run_dir, 'run_info.mat'), 'run_info');
    write_run_info(run_info, run_dir);

    if ~keep_open
        close(fig1);
        close(fig2);
        close(fig3);
        close(fig4);
    end
end

function write_run_info(run_info, run_dir)
    fid = fopen(fullfile(run_dir, 'run_info.txt'), 'w');
    fprintf(fid, 'Controller            : %s\n', char(run_info.controller));
    fprintf(fid, 'Single loop time      : %.3f s\n', run_info.single_loop_time_s);
    fprintf(fid, 'Max travel time       : %.3f s\n', run_info.max_travel_time_s);
    fprintf(fid, 'Goal reached          : %d\n', run_info.goal_reached);
    fprintf(fid, 'Timeout               : %d\n', run_info.timeout);
    fprintf(fid, 'Termination reason    : %s\n', char(run_info.termination_reason));
    fclose(fid);
end
