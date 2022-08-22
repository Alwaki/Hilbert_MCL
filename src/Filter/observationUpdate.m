function [particles] = observationUpdate(particles, measurements, model, ...
    beams_used, raycast_occupancy_limit, raycast_sampling_interval, ...
    raycast_max_length, sensor_variance, likelihood_model, ...
    point_mu, point_sigma)
% FUNCTION:     Calculates weights for and resamples particle set
%
% DESCRIPTION:  Updates the weight for each particle by relating
%               incoming range measurements with the map. For more detail
%               on method, see each individual likelihood model. After
%               weight update, weights are normalized and the effective
%               sample size (ESS) is calculated to determine if resampling
%               is necessary (by rule of thumb, at half particle_count 
%               resampling is considered necessary). See resampling 
%               function for more implementation detail.
%
% PARAMETERS:   particles: object handle, set of sample poses and weights
%               measurements: array of distances to objects
%               model: object handle, classifier instance
%
% RETURN:       particles: updated set of sample weights with same poses

% Calculate weights according to beam or point
if likelihood_model == 0
    particles = pointLikelihood(particles, measurements, model, beams_used, ...
        point_mu, point_sigma);
elseif likelihood_model == 1
    particles = beamLikelihood(particles, measurements, model, beams_used, ...
        raycast_occupancy_limit, raycast_sampling_interval, ...
        raycast_max_length, sensor_variance, 0);
elseif likelihood_model == 2
    particles = beamLikelihood(particles, measurements, model, beams_used, ...
        raycast_occupancy_limit, raycast_sampling_interval, ...
        raycast_max_length, sensor_variance, 1);
else
    disp("Invalid model, defaulting to point")
    particles = pointLikelihood(particles, measurements, model, beams_used, ...
        point_mu, point_sigma);
end

% Normalize weights
particles.weights = particles.weights ./ sum(particles.weights);

% Calculate ESS
ESS = 1 / dot(particles.weights,particles.weights);

% Resample depending on ESS
if ESS <= particles.particle_count/2
    %sprintf("ESS: %0.2d which is less than %0.2d, resampling!", ESS, particles.particle_count/2)
    particles = resample(particles);
end

end