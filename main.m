%% FORMALIA
%
% AUTHOR:           Alexander Wallen Kiessling
%
% Date:             June 2022
%
% DESCRIPTION:      This program simulates many runs of Monte Carlo
%                   Localization (MCL) throughout random 2D maps with a
%                   range based sensor. Different from traditional MCL with
%                   grid maps, this method utilizes a continuous map in the
%                   form of a classifier. Specifically, Hilbert maps are
%                   used. To use this program, edit the parameter file if
%                   needed, and then run this main file after ensuring that
%                   all program dependencies are met. 
%
% DEPENDENCIES:     Depends on navigation stack in Matlab, also utilizes
%                   the Hilbert map framework created in python by Lionel 
%                   Ott. As such there are some python dependencies, such
%                   as numpy, scipy, sklearn. 
%                   Note that to use this program Matlab will need to be 
%                   able to access python. Note compatibility between 
%                   specific versions of Matlab and python, and also that
%                   python aswell as libraries need to be accessible on
%                   path, otherwise code will throw errors when using
%                   classifier.

%% CODE

% Setup
addpath(genpath("include\"));
addpath(genpath("src\"));
setup;

% Main simulation loop
for iteration = 1:nbr_simulation_iterations
    data = tempname;
    % Generate data file for random map
    disp("Generating new map...")
    generateSimulatedData(sensor_min_range, sensor_max_range, sensor_range_noise, ...
        sensor_angle_noise, planner_max_iterations, planner_connection_distance, ...
        waypoints, data, odom_translational_noise, odom_angular_noise);
    
    % Parse data file as arrays
    [odom, ranges, ground_truth] = parse_carmen_file(data);
    
    % Create occupied point and free point vectors
    hitPoints = scans2points(ground_truth, ranges);
    freePoints = scans2freePoints(ground_truth, ranges);
    points = [hitPoints;freePoints];
    
    % Create labels for point vectors
    hitLabels = ones(size(hitPoints(:,1)));
    freeLabels = zeros(size(freePoints(:,1)));
    labels = [hitLabels;freeLabels];
    
    % Train python classifier with points and labels
    disp("Training model...")
    centers = py.util.sampling_coordinates(xlim, ylim, components);
    model = py.hilbert_map.SparseHilbertMap(centers, gamma, distance_cutoff);
    model.add(points, labels)
    
    % Create parameter container
    params = ParameterContainer(tracking_particle_count,...
                global_particle_count, beam_nbr, xlim, ylim, ...
                resolution, ground_truth, alpha, sensor_range_noise, sensor_noise_inflation, ...
                sensor_range_cutoff, raycast_occupancy_limit, raycast_sampling_interval, ...
                raycast_max_length, point_mu, point_sigma, ...
                beam_sensor_variance, likelihood_model, tracking_type, ...
                plotting_flag, inflate_sensor_noise, sensor_cutoff_flag);
    % Initialize localization filter
    disp("Simulating run...")
    tic
    [euc_error, heading_error] = simulateMCL(data, model, params);
    time = toc;
    % Append data collection
    if(length(euc_error) == length(ground_truth))
        ATE_pos(iteration) = rms(euc_error);
        ATE_rot(iteration) = rms(heading_error);
        mean_euclidean_errors(iteration) =  mean(euc_error);
        std_euclidean_errors(iteration) = std(euc_error);
        mean_heading_errors(iteration) = mean(heading_error);
        std_heading_errors(iteration) = std(heading_error);
        mean_times(iteration) = time/length(euc_error);
        success_count = success_count + 1;
        disp("Tracking finished, adding data")
    end
end

