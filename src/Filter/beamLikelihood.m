function [particles] = beamLikelihood(particles, measurements, angles, model, ...
    params)
%beamLikelihood calculates likelihood of given measurements
%       Uses the beam likelihood model to calculate likelihoods
%       for observations.

% Assume continuous, evenly spaced measurement list
measurement_count = length(measurements);

% Pre-allocate containers for speed
p_list = zeros(particles.particle_count, measurement_count);
expected_distances = zeros(particles.particle_count, measurement_count);

for i = 1:particles.particle_count
    for j = 1:measurement_count
        heading = wrapToPi(particles.poses(i,3) + angles(j));
        distance = raycast(model, particles.poses(i,1:2), heading, ...
            params);
        expected_distances(i,j) = distance;
    end
end

n = 1.0 / sqrt(2 * pi * params.beam_sensor_variance);
real_distances = measurements;
real_distances = repmat(real_distances, particles.particle_count, 1);
diff = (real_distances-expected_distances);
diff = diff.^2;
diff = diff./(2*n);
p_list(:,:) = n*exp(-diff);


% Multiply independent likelihoods
LL = sum(log(p_list), 2);
lse = max(LL) + log(sum(exp(LL-max(LL))));
w = exp(LL - lse); 
particles.weights = w;

%particles.weights = prod(p_list,2);

end