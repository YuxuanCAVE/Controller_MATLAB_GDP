function yaw_ref = path_yaw(x_ref, y_ref)
    dx = gradient(x_ref);
    dy = gradient(y_ref);
    yaw_ref = unwrap(atan2(dy, dx));
end
