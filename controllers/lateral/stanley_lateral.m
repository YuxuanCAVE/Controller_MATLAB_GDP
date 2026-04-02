function delta = stanley_lateral(x, y, yaw, v, ref, L, p, idx_hint, window)
    if nargin < 8
        idx_hint = [];
    end
    if nargin < 9
        window = [];
    end

    [idx, ~, ~, psi_ref, e_ct, seg_idx, seg_t] = nearest_path_ref_point( ...
        x, y, ref.x, ref.y, idx_hint, window);
    kappa = interpolate_projected_curvature(ref.kappa, idx, seg_idx, seg_t);
    e_psi = angle_wrap(psi_ref - yaw);

    delta_ff = p.delta_ff_gain * atan(L * kappa);
    delta_fb = p.k_yaw * e_psi + atan2(p.k_cte * e_ct, p.k_soft + max(v, 0.5));

    delta = delta_ff + delta_fb;
end

function kappa = interpolate_projected_curvature(kappa_ref, idx, seg_idx, seg_t)
    n = numel(kappa_ref);
    if ~isnan(seg_idx) && seg_idx >= 1 && seg_idx < n
        kappa = (1 - seg_t) * kappa_ref(seg_idx) + seg_t * kappa_ref(seg_idx + 1);
    else
        kappa = kappa_ref(idx);
    end
end
