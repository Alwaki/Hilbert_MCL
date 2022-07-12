function [particles] = pointLikelihood(particles, measurements, model, ...
    beams_used, mu, sigma)
% FUNCTION:     Calculates likelihood of given measurements
%
% DESCRIPTION:  
%
% PARAMETERS:   particles: object handle, set of sample poses and weights
%               measurements: array of distances to objects
%               model: object handle, classifier instance
%               beams_used: number of measurements used, rest are ignored
%               mu: mean of truncated normal distribution for likelihood
%               sigma: standard deviation of truncated normal distribution
%
% RETURN:       particles: updated set with new weights, same poses

% Create truncated normal distribution
normal = makedist('Normal', mu, sigma);
truncated_normal = truncate(normal, 0, 1);

% Reduce measurement count to specified number
interval = floor(length(measurements(:))/beams_used);
measurements = measurements(:,1:interval:end);

% Assume continuous, evenly spaced measurement list
measurement_count = length(measurements);
angle_increment = pi / (measurement_count-1);

% Pre-allocate container for speed
p_list = zeros(particles.particle_count, measurement_count);

% Classify points and assign likelihoods
for i = 1:measurement_count
    headings = wrapToPi(particles.poses(:,3) + (i-1) * angle_increment - (pi/2));
    points_x = particles.poses(:,1) + cos(headings) .* measurements(i);
    points_y = particles.poses(:,2) + sin(headings) .* measurements(i);
    p = double(model.classify([points_x, points_y]));
    p = p(:,2); 
    p_list(:,i) = pdf(truncated_normal, p);
end

% Multiply independent likelihoods to yield weight
particles.weights = prod(p_list,2);

end