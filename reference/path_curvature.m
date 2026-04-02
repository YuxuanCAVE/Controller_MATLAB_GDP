function kappa = path_curvature(x_ref, y_ref)
    dx = gradient(x_ref);
    dy = gradient(y_ref);
    ddx = gradient(dx);
    ddy = gradient(dy);
    den = (dx.^2 + dy.^2).^(3/2);
    den(den < 1e-9) = 1e-9;
    kappa = (dx .* ddy - dy .* ddx) ./ den;
    kappa(~isfinite(kappa)) = 0;
end
