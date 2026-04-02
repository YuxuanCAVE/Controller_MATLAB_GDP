function ref = load_reference_path(path_file)
    S = load(path_file);
    ref.x = S.x_opt(:);
    ref.y = S.y_opt(:);

    if numel(ref.x) >= 2 && hypot(ref.x(end) - ref.x(1), ref.y(end) - ref.y(1)) < 0.50
        ref.x = ref.x(1:end-1);
        ref.y = ref.y(1:end-1);
    end

    ref.yaw = path_yaw(ref.x, ref.y);
    ref.kappa = path_curvature(ref.x, ref.y);
end
