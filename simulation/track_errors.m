function [idx, e_ct, e_psi] = track_errors(x, y, yaw, ref, idx_hint, window)
    if nargin < 5
        idx_hint = [];
    end
    if nargin < 6
        window = [];
    end

    [idx, ~, ~, psi_ref, ey_proj] = nearest_path_ref_point( ...
        x, y, ref.x, ref.y, idx_hint, window);

    e_ct = ey_proj;
    e_psi = angle_wrap(yaw - psi_ref);
end
