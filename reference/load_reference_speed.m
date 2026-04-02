function ref = load_reference_speed(ref, speed_cfg)
    switch speed_cfg.mode
        case "constant"
            ref.v_ref = speed_cfg.constant_value * ones(size(ref.x));
            ref.v_ref_mode = "constant";

        case "profile"
            S = load(speed_cfg.profile_file);
            if ~isfield(S, 'pathv_ref')
                error('Speed profile file must contain pathv_ref: %s', speed_cfg.profile_file);
            end

            % The velocity profile mat contains pathv_ref as a 1x389 array.
            % Due to the path ordering/alignment used by this project,
            % only samples 26:384 are valid for the current reference path.
            v_profile = S.pathv_ref(26:384).';

            if numel(v_profile) ~= numel(ref.x)
                error(['Speed profile length mismatch after slicing pathv_ref(26:384). ', ...
                       'Expected %d samples, got %d from file: %s'], ...
                       numel(ref.x), numel(v_profile), speed_cfg.profile_file);
            end

            ref.v_ref = v_profile;

            ref.v_ref_mode = "profile";
            ref.v_ref_source = speed_cfg.profile_file;

        otherwise
            error('Unknown speed reference mode: %s', speed_cfg.mode);
    end

    ref.v_ref = ref.v_ref(:);
end
