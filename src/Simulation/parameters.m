%% PARAMETER FILE

% Simulation settings
nbr_simulation_iterations = 1;      % Number of runs that will be simulated

% Motion model parameters
alpha = [0.1, 0.01, 0.1, 0.01];     % Motion noise in odometry model

% Likelihood model parameters
likelihood_model = 0;               % 0: point, 1: beam, 2: beam_ip
raycast_occupancy_limit = 0.5;      
raycast_sampling_interval = 0.1;
raycast_max_length = 20;
beam_sensor_variance = 0.1;         % Variance used in beam model
point_mu = 0.5;                     % Mean in LFM model
point_sigma = 0.1;                  % STD in LFM model

% Map parameters
xlim = [0.0, 20.0];                      % Map borders
ylim = [0.0, 10.0];
resolution = 0.05;                       % Map resolution in heatmap

% Map generation parameters
sensor_max_range = 40;
sensor_min_range = 0;
sensor_range_noise = 0.05;
sensor_angle_noise = 0.001;
planner_max_iterations = 400000;
planner_connection_distance = 0.5;
waypoints = [2.5, 2.0, 0; 17.5, 8.0, pi/2];  % Start -> End for planner
odom_translational_noise = 0.1;              % Simulated odom noise
odom_angular_noise = 0.01; 

% Classifier parameters
components = int16(100);
gamma = 8;
distance_cutoff = 0.001;

% Filter parameters (MCL)
tracking_type = 0;                       % 0: local, 1: global
tracking_particle_count = 200;
global_particle_count = 5000;
beam_nbr = 20;
sensor_range_cutoff = 20;
sensor_noise_inflation = 0.1;
sensor_cutoff_flag = 0;                  % 0: no removal, 1: remove at cutoff
inflate_sensor_noise = 1;                % 0: normal, 1: add 10x noise
plotting_flag = 1;                       % 0: No plotting, 1: plotting enabled
