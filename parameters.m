%% PARAMETER FILE

% Motion model parameters
alpha = [0.1, 0.01, 0.1, 0.01];

% Likelihood model parameters
likelihood_model = 0;                    % 0: beam, 1: point
raycast_occupancy_limit = 0.5;
raycast_sampling_interval = 0.1;
raycast_max_length = 20;
sensor_variance = 0.01;
point_mu = 0.5;
point_sigma = 0.1;

% Map parameters
data = "corrected_simulated_map.txt";    % Map file name
xlim = [0.0, 20.0];                      % Borders
ylim = [0.0, 10.0];
resolution = 0.05;

% Map generation parameters
sensor_max_range = 40;
sensor_min_range = 0;
sensor_range_noise = 0.05;
sensor_angle_noise = 0.001;
planner_max_iterations = 40000;
planner_connection_distance = 0.5;
waypoints = [2.5, 2.0, 0; 17.5, 8.0, pi/2];
odom_translational_noise = 0.1; %0.01;
odom_angular_noise = 0.01; %0.001;

% Use these waypoints instead for full map data, if needed
% waypoints = [2.5, 2.0, 0; 8.5, 2.5, 0; 11.5, 2.5, 0;
%     17.5, 2.0, -pi/2; 17.5, 8.0, pi/2; 11.5, 8.0, pi;
%     8.5, 8.0, pi; 2.5, 8.0, pi];

% Classifier parameters
components = int16(100);
gamma = 8;
distance_cutoff = 0.001;

% Filter parameters (MCL)
tracking_type = 0;                       % 0: local, 1: global
particle_nbr = 200;
beam_nbr = 20;
plotting_flag = 1;                       % 0: No plotting, 1: plotting enabled