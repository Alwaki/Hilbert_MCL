function generateSimulatedData(sensor_min_range, sensor_max_range, sensor_range_noise, ...
    sensor_angle_noise, planner_max_iterations, ...
    planner_connection_distance, waypoints, filename, ...
    odom_translational_noise, odom_angular_noise)
% FUNCTION:     Generates simulated odom and range data file
%
% Description:  This program is intended to run through an environment
%               and generate range measurements. It will generate a random
%               map, and write these to file in CARMEN format style. The
%               map itself is a maze grid occupancy map with very fine
%               resolution. To generate feasible measurements, RRT* is used
%               to traverse the environment in a path. At intervals of this
%               path, odometry and range measurements are generated with
%               noise.
%
% PARAMETERS:   see parameter file
%

% Create 2D binary occupancy map with maze obstacles
map = mapMaze(100,50,'MapSize',[20 10],'MapResolution',50);
inflated_map = copy(map);
inflate(inflated_map,0.3);

% Specify sensor settings
rbsensor = rangeSensor('Range',[sensor_min_range sensor_max_range], ...
    'HorizontalAngle',[-pi/2 pi/2], ...
    'HorizontalAngleResolution', pi/360, 'RangeNoise', sensor_range_noise, ...
    'HorizontalAngleNoise', sensor_angle_noise);

% Create 2D state space 
ss = stateSpaceSE2([0,20;0,10;-pi,pi]);
ss.WeightTheta = 0;

% Planning validation
validator = validatorOccupancyMap(ss);
validator.Map = inflated_map;
validator.ValidationDistance = 0.1;

% Create planner
planner = plannerRRTStar(ss, validator);
planner.MaxConnectionDistance = planner_connection_distance;
planner.MaxIterations = planner_max_iterations;
planner.ContinueAfterGoalReached = false;

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
    x = full_path(i,1) + normrnd(0,odom_translational_noise);
    y = full_path(i,2) + normrnd(0,odom_translational_noise);
    theta = full_path(i,3) + normrnd(0,odom_angular_noise);
    odom = [odom;[x,y,theta]];
end

%% EXPORT TO CARMEN STYLE FILE
fileID = fopen(filename,'w');
for i = 1:length(full_path)
    fprintf(fileID,'ODOM');
    fprintf(fileID,' %f %f',  odom(i,:), [0,0,0]);
    fprintf(fileID,'\n');
    fprintf(fileID,'FLASER');
    fprintf(fileID,' %d %f %f %f',  length(full_ranges(:,i)), full_ranges(:,i), full_path(i,:), full_path(i,:));
    fprintf(fileID,'\n');
end
fclose(fileID);



end