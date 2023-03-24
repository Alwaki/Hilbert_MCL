function [particles] = pointLikelihood(particles, measurements, angles, model, ...
    params)
%pointLikelihood calculates likelihood of given measurements
%       Uses the modified likelihood field model to calculate 
%       likelihoods of observations at discrete points.

% Create truncated normal distribution
normal = makedist('Normal', params.point_mu, params.point_sigma);
truncated_normal = truncate(normal, 0, 1);

% Pre-allocate container for speed
measurement_count = length(measurements);
p_list = zeros(particles.particle_count, measurement_count);

% Classify points and assign likelihoods
for i = 1:measurement_count
    headings = wrapToPi(particles.poses(:,3) + angles(i));
    points_x = particles.poses(:,1) + cos(headings) .* measurements(i);
    points_y = particles.poses(:,2) + sin(headings) .* measurements(i);
    p = double(model.classify([points_x, points_y]));
    p = p(:,2); 
    p_list(:,i) = pdf(truncated_normal, p);
end

% Multiply independent likelihoods to yield weight
LL = sum(log(p_list), 2);
lse = max(LL) + log(sum(exp(LL-max(LL))));
w = exp(LL - lse); 
particles.weights = w;

%particles.weights = prod(p_list,2);

end