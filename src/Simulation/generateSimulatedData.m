function [data, measurement_angles, model, full_path] = generateSimulatedData(params)
% generateSimulatedData  Generates simulated odom and range data file
%       This program is intended to run through an environment
%       and generate range measurements. It will generate a random
%       map, and write these to file in CARMEN format style. The
%       map itself is a maze grid occupancy map with very fine
%       resolution. To generate feasible measurements, Hybrid A* is used
%       to traverse the environment in a path. At intervals of this
%       path, odometry and range measurements are generated with
%       noise in two sets, one for mapping and one for localization.

% Set temporary name (helps for parallel data handling)
data = tempname;
disp("Generating new map...")

% Create 2D binary occupancy map with maze structure
map = mapMaze(100,50,'MapSize',[20 10],'MapResolution',50);
inflated_map = copy(map);
inflate(inflated_map,0.3);

% Specify sensor settings
rbsensor_map = rangeSensor('Range',[params.sensor_min_range params.sensor_max_range], ...
    'HorizontalAngle',[-pi pi], ...
    'HorizontalAngleResolution', 2*pi/360);

rbsensor_partial = rangeSensor('Range',[params.sensor_min_range params.sensor_max_range], ...
    'HorizontalAngle',[-pi pi], ...
    'HorizontalAngleResolution', 2*pi/20, 'RangeNoise', params.sensor_range_noise, ...
    'HorizontalAngleNoise', params.sensor_angle_noise);

% % Create 2D state space 
ss = stateSpaceSE2([0,20;0,10;-pi,pi]);
ss.WeightTheta = 0;

% Planning validation
validator = validatorOccupancyMap(ss);
validator.Map = inflated_map;
validator.ValidationDistance = 0.1;

% Create planner
planner = plannerHybridAStar(validator, 'MinTurningRadius',0.64, ...
    'MotionPrimitiveLength',1, "InterpolationDistance", 0.5);
waypoints = params.waypoints;

% Generate path
start = waypoints(1,:);
goal = waypoints(end,:);
full_path = [];
while length(full_path) < 1
    ref_path = plan(planner, start, goal);
    full_path = ref_path.States;
end

% Create range measurements for map along path
full_ranges_map = [];
full_angles_map = [];
for i = 1:length(full_path)
    [ranges, angles] = rbsensor_map(full_path(i,:), map);
    full_ranges_map = [full_ranges_map ranges];
    full_angles_map = [full_angles_map; angles'];
end

% Create range measurements for localization along path
full_ranges_partial = [];
full_angles_partial = [];
for i = 1:length(full_path)
    [ranges, angles] = rbsensor_partial(full_path(i,:), map);
    full_ranges_partial = [full_ranges_partial ranges];
    full_angles_partial = [full_angles_partial; angles'];
end
measurement_angles = full_angles_partial;

% Create pose measurements with noise along path
odom = [];
for i = 1:length(full_path)
    x = full_path(i,1) + normrnd(0,params.odom_translational_noise);
    y = full_path(i,2) + normrnd(0,params.odom_translational_noise);
    theta = full_path(i,3) + normrnd(0,params.odom_angular_noise);
    odom = [odom;[x,y,theta]];
end

%% EXPORT TO CARMEN STYLE FILE

% Partial measurements
fileID = fopen(data,'w');
for i = 1:length(full_path)
    fprintf(fileID,'ODOM');
    fprintf(fileID,' %f %f',  odom(i,:), [0,0,0]);
    fprintf(fileID,'\n');
    fprintf(fileID,'FLASER');
    fprintf(fileID,' %d %f %f %f',  length(full_ranges_partial(:,i)), full_ranges_partial(:,i), full_path(i,:), full_path(i,:));
    fprintf(fileID,'\n');
end
fclose(fileID);

data_map = tempname;

% Partial measurements
fileID = fopen(data_map,'w');
for i = 1:length(full_path)
    fprintf(fileID,'ODOM');
    fprintf(fileID,' %f %f',  odom(i,:), [0,0,0]);
    fprintf(fileID,'\n');
    fprintf(fileID,'FLASER');
    fprintf(fileID,' %d %f %f %f',  length(full_ranges_map(:,i)), full_ranges_map(:,i), full_path(i,:), full_path(i,:));
    fprintf(fileID,'\n');
end
fclose(fileID);

[~, full_ranges_map, ~] = parse_carmen_file(data_map);

% Create occupied point and free point vectors
hitPoints = scans2points(full_path, full_ranges_map, full_angles_map);
freePoints = scans2freePoints(full_path, full_ranges_map, full_angles_map);
points = [hitPoints;freePoints];

% Create labels for point vectors
hitLabels = ones(size(hitPoints(:,1)));
freeLabels = zeros(size(freePoints(:,1)));
labels = [hitLabels;freeLabels];

% Train python classifier with points and labels
disp("Training model...")
centers = py.util.sampling_coordinates(params.xlim, params.ylim, params.components);
model = py.hilbert_map.SparseHilbertMap(centers, params.gamma, params.distance_cutoff);
model.add(points, labels)

end