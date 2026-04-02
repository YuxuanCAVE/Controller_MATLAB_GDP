function [idx, xr, yr, psi_r, ey, seg_idx, seg_t] = nearest_path_ref_point(x, y, x_ref, y_ref, idx_hint, window)
% Find the nearest reference point on a path using:
%   1) nearest waypoint search
%   2) candidate segment projection
%
% Inputs:
%   x, y       - vehicle rear axle center position
%   x_ref      - reference path x coordinates
%   y_ref      - reference path y coordinates
%   idx_hint   - previous nearest index (optional)
%   window     - local search window size (optional)
%
% Outputs:
%   idx        - nearest waypoint index (coarse)
%   xr, yr     - projected reference point on nearest segment
%   psi_r      - heading of the chosen segment
%   ey         - signed lateral error
%   seg_idx    - start index of the chosen segment
%   seg_t      - normalized projection position on the chosen segment

    n = numel(x_ref);
    if n < 2
        error('Path must contain at least 2 waypoints.');
    end

    % ------------------------------------------------------------
    % Step 1: nearest waypoint search (coarse index)
    % ------------------------------------------------------------
    if nargin < 5 || isempty(idx_hint)
        d2 = (x_ref - x).^2 + (y_ref - y).^2;
        [~, idx] = min(d2);
    else
        if nargin < 6 || isempty(window)
            window = 40;
        end

        i0 = max(1, idx_hint - window);
        i1 = min(n, idx_hint + window);

        d2 = (x_ref(i0:i1) - x).^2 + (y_ref(i0:i1) - y).^2;
        [~, k] = min(d2);
        idx = i0 + k - 1;
    end

    % ------------------------------------------------------------
    % Step 2: build candidate segments around idx
    % ------------------------------------------------------------
    candidates = [];

    if idx > 1
        candidates = [candidates; idx-1, idx];
    end
    if idx < n
        candidates = [candidates; idx, idx+1];
    end

    % Safety fallback (should not happen if n >= 2)
    if isempty(candidates)
        error('No valid candidate segment found.');
    end

    % ------------------------------------------------------------
    % Step 3: project vehicle point onto each candidate segment
    %         and choose the closest projection
    % ------------------------------------------------------------
    best_dist2 = inf;
    xr = NaN;
    yr = NaN;
    psi_r = NaN;
    ey = NaN;
    seg_idx = NaN;
    seg_t = NaN;

    for m = 1:size(candidates, 1)
        iA = candidates(m, 1);
        iB = candidates(m, 2);

        Ax = x_ref(iA);
        Ay = y_ref(iA);
        Bx = x_ref(iB);
        By = y_ref(iB);

        ABx = Bx - Ax;
        ABy = By - Ay;
        APx = x - Ax;
        APy = y - Ay;

        denom = ABx^2 + ABy^2;
        if denom < 1e-12
            continue;
        end

        % Projection parameter
        t = (APx * ABx + APy * ABy) / denom;
        t = min(1.0, max(0.0, t));

        % Projection point
        xp = Ax + t * ABx;
        yp = Ay + t * ABy;

        % Squared distance from vehicle to projection
        dx = x - xp;
        dy = y - yp;
        dist2 = dx^2 + dy^2;

        if dist2 < best_dist2
            best_dist2 = dist2;

            xr = xp;
            yr = yp;
            seg_idx = iA;
            seg_t = t;

            % Segment heading
            psi_r = atan2(ABy, ABx);

            % Signed lateral error
            % ey = [-sin(psi_r), cos(psi_r)] * [dx; dy]
            ey = -sin(psi_r) * dx + cos(psi_r) * dy;
        end
    end

    % Optional fallback if all candidate segments were degenerate
    if isnan(xr)
        xr = x_ref(idx);
        yr = y_ref(idx);

        if idx < n
            psi_r = atan2(y_ref(idx+1) - y_ref(idx), x_ref(idx+1) - x_ref(idx));
            seg_idx = idx;
            seg_t = 0;
        else
            psi_r = atan2(y_ref(idx) - y_ref(idx-1), x_ref(idx) - x_ref(idx-1));
            seg_idx = idx - 1;
            seg_t = 1;
        end

        dx = x - xr;
        dy = y - yr;
        ey = -sin(psi_r) * dx + cos(psi_r) * dy;
    end
end
