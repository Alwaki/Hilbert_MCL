function [particles] = beamLikelihood(particles, measurements, model, ...
    beams_used, raycast_occupancy_limit, raycast_sampling_interval, ...
    raycast_max_length, sensor_variance, interpolation_flag)
% FUNCTION:     Calculates likelihood of given measurements
%
% DESCRIPTION:  
%
% PARAMETERS:   particles: object handle, set of sample poses and weights
%               measurements: array of distances to objects
%               model: object handle, classifier instance
%               beams_used: number of measurements used, rest are ignored
%               raycast_occupancy_limit: required occupancy to terminate
%               raycast_sampling_interval: distance between each occupancy
%               check
%               raycast_max_length: maximum checked distance before
%               terminating
%               sensor_variance: accuracy variance of range measurements
%
% RETURN:       particles: updated set with new weights, same poses

% Reduce measurement count to specified number
interval = floor(length(measurements(:))/beams_used);
measurements = measurements(:,1:interval:end);

% Assume continuous, evenly spaced measurement list
measurement_count = length(measurements);
angle_increment = pi / (measurement_count-1);

% Pre-allocate containers for speed
p_list = zeros(particles.particle_count, measurement_count);
expected_distances = zeros(particles.particle_count, measurement_count);

for i = 1:particles.particle_count
    for j = 1:measurement_count
        heading = wrapToPi(particles.poses(i,3) + (j-1) * angle_increment - (pi/2));
        distance = raycast(model, particles.poses(i,1:2), heading, ...
            raycast_occupancy_limit, raycast_sampling_interval, ...
            raycast_max_length, interpolation_flag);
        expected_distances(i,j) = distance;
    end
end

n = 1.0 / sqrt(2 * pi * sensor_variance);
real_distances = measurements;
real_distances = repmat(real_distances, particles.particle_count, 1);
diff = (real_distances-expected_distances);
diff = diff.^2;
diff = diff./(2*n);
p_list(:,:) = n*exp(-diff);


% Multiply independent likelihoods
particles.weights = prod(p_list,2);

end