classdef ParameterContainer
    % Intended to enable passing parameters by struct ref

    properties
        % Filter parameters
        tracking_particle_count;
        global_particle_count;
        beam_nbr;

        % Map & navigation parameters
        xlim;
        ylim;
        resolution;
        gt;

        % Motion & sensor parameters
        alpha;
        sensor_range_noise;
        sensor_noise_inflation;
        sensor_range_cutoff;

        % Observation parameters
        raycast_occupancy_limit;
        raycast_sampling_interval;
        raycast_max_length;
        point_mu;
        point_sigma;
        beam_sensor_variance;

        % Boolean flags
        likelihood_model;
        tracking_type;
        plotting_flag;
        inflate_noise;
        sensor_cutoff_flag;
    end

    methods
        function obj = ParameterContainer(tracking_particle_count,...
                global_particle_count, beam_nbr, xlim, ylim, ...
                resolution, gt, alpha, sensor_range_noise, sensor_noise_inflation, ...
                sensor_range_cutoff, raycast_occupancy_limit, raycast_sampling_interval, ...
                raycast_max_length, point_mu, point_sigma, ...
                beam_sensor_variance, likelihood_type, tracking_type, ...
                plotting_flag, inflate_noise, sensor_cutoff_flag)
            % Construction
            obj.tracking_particle_count = tracking_particle_count;
            obj.global_particle_count = global_particle_count;
            obj.beam_nbr = beam_nbr;
            obj.xlim = xlim;
            obj.ylim = ylim;
            obj.resolution = resolution;
            obj.gt = gt;
            obj.alpha = alpha;
            obj.sensor_range_noise = sensor_range_noise;
            obj.sensor_noise_inflation = sensor_noise_inflation;
            obj.sensor_range_cutoff = sensor_range_cutoff;
            obj.raycast_occupancy_limit = raycast_occupancy_limit;
            obj.raycast_sampling_interval = raycast_sampling_interval;
            obj.raycast_max_length = raycast_max_length;
            obj.point_mu = point_mu;
            obj.point_sigma = point_sigma;
            obj.beam_sensor_variance = beam_sensor_variance;
            obj.likelihood_model = likelihood_type;
            obj.tracking_type = tracking_type;
            obj.plotting_flag = plotting_flag;
            obj.inflate_noise = inflate_noise;
            obj.sensor_cutoff_flag = sensor_cutoff_flag;
        end


    end
end