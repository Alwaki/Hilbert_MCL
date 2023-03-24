classdef ParameterContainer
    % ParameterContainer used to pass parameters by struct ref

    properties
        % Run parameters
        nbr_simulation_iterations

        % Filter parameters
        tracking_particle_count;
        global_particle_count;

        % Map parameters
        xlim;
        ylim;
        resolution;

        % Motion & sensor parameters
        alpha;
        sensor_noise_inflation;
        sensor_range_cutoff;
        sensor_max_range;
        sensor_min_range;
        sensor_range_noise;
        sensor_angle_noise;
        waypoints;
        odom_translational_noise;           
        odom_angular_noise;

        % Observation parameters
        raycast_occupancy_limit;
        raycast_sampling_interval;
        raycast_max_length;
        point_mu;
        point_sigma;
        beam_sensor_variance;

        % Classifier parameters
        components;
        gamma;
        distance_cutoff;

        % Boolean flags
        likelihood_model;
        tracking_type;
        plotting_flag;
        inflate_noise;
        sensor_cutoff_flag;
    end

    methods
        % Constructor
        function obj = ParameterContainer(nbr_simulation_iterations, tracking_particle_count,...
                global_particle_count, xlim, ylim, ...
                resolution, alpha, sensor_max_range, sensor_min_range, ...
                sensor_range_noise, sensor_angle_noise, waypoints, ...
                odom_translational_noise, odom_angular_noise, sensor_noise_inflation, ...
                sensor_range_cutoff, raycast_occupancy_limit, raycast_sampling_interval, ...
                raycast_max_length, point_mu, point_sigma, ...
                beam_sensor_variance, likelihood_model, tracking_type, ...
                plotting_flag, inflate_noise, sensor_cutoff_flag, components, ...
                gamma, distance_cutoff)
            
            obj.nbr_simulation_iterations = nbr_simulation_iterations;
            obj.tracking_particle_count = tracking_particle_count;
            obj.global_particle_count = global_particle_count;
            obj.xlim = xlim;
            obj.ylim = ylim;
            obj.resolution = resolution;
            obj.alpha = alpha;
            obj.sensor_range_noise = sensor_range_noise;
            obj.sensor_max_range = sensor_max_range;
            obj.sensor_min_range = sensor_min_range;
            obj.sensor_range_noise = sensor_range_noise;
            obj.sensor_angle_noise = sensor_angle_noise;
            obj.waypoints = waypoints;
            obj.odom_translational_noise = odom_translational_noise;           
            obj.odom_angular_noise = odom_angular_noise;
            obj.sensor_noise_inflation = sensor_noise_inflation;
            obj.sensor_range_cutoff = sensor_range_cutoff;
            obj.raycast_occupancy_limit = raycast_occupancy_limit;
            obj.raycast_sampling_interval = raycast_sampling_interval;
            obj.raycast_max_length = raycast_max_length;
            obj.point_mu = point_mu;
            obj.point_sigma = point_sigma;
            obj.beam_sensor_variance = beam_sensor_variance;
            obj.likelihood_model = likelihood_model;
            obj.tracking_type = tracking_type;
            obj.plotting_flag = plotting_flag;
            obj.inflate_noise = inflate_noise;
            obj.sensor_cutoff_flag = sensor_cutoff_flag;
            obj.components = components;
            obj.gamma = gamma;
            obj.distance_cutoff = distance_cutoff;
        end


    end
end