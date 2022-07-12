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

%% SETUP

% Clean workspace
clc; clear; close all;

% Load parameter file
parameters;


%% CODE

% Data collection structures
euclidean_errors_cell = {};
heading_errors_cell = {};

iteration = 1;
run_counter = 0;
while iteration < 21
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
    
    % Initialize localization filter
    disp("Simulating run...")
    tic
    [euc_error, heading_error] = simulateMCL(data, model, tracking_type, ...
        particle_nbr, xlim, ylim, alpha, ground_truth, resolution, plotting_flag, ...
        raycast_occupancy_limit, raycast_sampling_interval, raycast_max_length, ...
        beam_nbr, sensor_variance, likelihood_model, point_mu, point_sigma);
    toc
    % Append data collection
    if(length(euc_error) == length(ground_truth))
        euclidean_errors_cell{iteration} =  euc_error;
        heading_errors_cell{iteration} = heading_error;
        iteration = iteration + 1;
    end

    run_counter = run_counter + 1;
    disp(run_counter)

end



