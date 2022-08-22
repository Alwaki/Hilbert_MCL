% Clean workspace
clc; clear; close all;

% Load parameter file
parameters;

% Create data collection structures
ATE_pos = zeros(nbr_simulation_iterations, 1);
ATE_rot = zeros(nbr_simulation_iterations, 1);
mean_euclidean_errors = zeros(nbr_simulation_iterations,1);
std_euclidean_errors = zeros(nbr_simulation_iterations,1);
mean_heading_errors = zeros(nbr_simulation_iterations,1);
std_heading_errors = zeros(nbr_simulation_iterations,1);
mean_times = zeros(nbr_simulation_iterations,1);
success_count = 0;