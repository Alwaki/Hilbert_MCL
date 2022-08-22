%% FORMALIA

% Project:      Ranged data generation simulation
% Author:       Alexander Wallen Kiessling
% Description:  This program is intended to run through an environment
%               and generate range measurements.
%

%% SETUP

% Cleanup
clc; clear; close all;

% Create 2D binary occupancy map with maze obstacles
% map = mapMaze(100,50,'MapSize',[20 10],'MapResolution',50);
% inflated_map = copy(map);
% inflate(inflated_map,0.25);

map = mapMaze(100,50,'MapSize',[20 10],'MapResolution',50);
figure(1)
show(map)
inflated_map = copy(map);
inflate(inflated_map,0.3);

% Specify sensor settings
rbsensor = rangeSensor('Range',[0 40], 'HorizontalAngle',[-pi/2 pi/2], ...
    'HorizontalAngleResolution', pi/360, 'RangeNoise', 0.05, ...
    'HorizontalAngleNoise', 0.001);

% Create 2D state space 
ss = stateSpaceSE2([0,20;0,10;-pi,pi]);
ss.WeightTheta = 0;

% Planning validation
validator = validatorOccupancyMap(ss);
validator.Map = inflated_map;
validator.ValidationDistance = 0.1;

% Create planner
planner = plannerRRTStar(ss, validator);
planner.MaxConnectionDistance = 0.5;
planner.MaxIterations = 40000;
planner.ContinueAfterGoalReached = false;

% Waypoints
% waypoints = [2.5, 2.0, 0; 8.5, 2.5, 0; 11.5, 2.5, 0;
%     17.5, 2.0, -pi/2; 17.5, 8.0, pi/2; 11.5, 8.0, pi;
%     8.5, 8.0, pi; 2.5, 8.0, pi];
waypoints = [2.5, 2.0, 0; 17.5, 8.0, pi/2];

%% EXECUTION

% Generate path
current = waypoints(1,:);
full_path = [];
for i = 2:length(waypoints(:,1))
    goal = waypoints(i,:);
    path = plan(planner, current, goal);
    full_path = [full_path; path.States];
    current = goal;
end

% Create range measurements along path
full_ranges = [];
full_angles = [];
for i = 1:length(full_path)
[ranges, angles] = rbsensor(full_path(i,:), map);
full_ranges = [full_ranges ranges];
full_angles = [full_angles angles];
end

% Create pose measurements with noise along path
odom = [];
for i = 1:length(full_path)
x = full_path(i,1) + normrnd(0,0.1);
y = full_path(i,2) + normrnd(0,0.1);
theta = full_path(i,3) + normrnd(0,0.01);
odom = [odom;[x,y,theta]];
end

% Plot paths
% figure
% plot(full_path(:,1),full_path(:,2),'b-');
% hold on
% plot(odom(:,1),odom(:,2),'r-');
% hold off

%% EXPORT TO CARMEN STYLE FILE
fileID = fopen('corrected_simulated_map.txt','w');
for i = 1:length(full_path)
fprintf(fileID,'ODOM');
fprintf(fileID,' %f %f',  odom(i,:), [0,0,0]);
fprintf(fileID,'\n');
fprintf(fileID,'FLASER');
fprintf(fileID,' %d %f %f %f',  length(full_ranges(:,i)), full_ranges(:,i), full_path(i,:), full_path(i,:));
fprintf(fileID,'\n');
end
fclose(fileID);

data = "corrected_simulated_map.txt";    % Map file name
xlim = [0.0, 20.0];                      % Borders
ylim = [0.0, 10.0];
resolution = 0.05;
% Classifier parameters
components = int16(100);
gamma = 8;
distance_cutoff = 0.001;

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

visualizeScans(freePoints, hitPoints, ground_truth)
visualizeHeatmap(model, xlim, ylim, resolution)