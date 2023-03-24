function [particles, time] = observationUpdate(particles, measurements, angles, ...
    model, params)
% observationUpdate calculates weights for and resamples particle set
%             Updates the weight for each particle by relating
%             incoming range measurements with the map. For more detail
%             on method, see each individual likelihood model. After
%             weight update, weights are normalized and the effective
%             sample size (ESS) is calculated to determine if resampling
%             is necessary (by rule of thumb, at half particle_count 
%             resampling is considered necessary). See resampling 
%             function for more implementation detail.


% Calculate weights according to beam or point
tic; 
if params.likelihood_model == 0
    particles = pointLikelihood(particles, measurements, angles, model, ...
        params);
elseif params.likelihood_model == 1
    particles = beamLikelihood(particles, measurements, angles, model, ...
        params);
else
    % Default to point likelihood, display warning
    warning('Invalid likelihood model selected. Defaulting to pointLikelihood.')
    particles = pointLikelihood(particles, measurements, angles, model, ...
        params);
end
time = toc;

% Normalize weights
% particles.weights = particles.weights ./ sum(particles.weights);

% Calculate ESS
ESS = 1 / dot(particles.weights,particles.weights);

% Resample depending on ESS
if ESS <= particles.particle_count/2
    %sprintf("ESS: %0.2d which is less than %0.2d, resampling!", ESS, particles.particle_count/2)
    particles = resample(particles);
end

end