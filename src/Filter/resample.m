function [particles] = resample(particles)
% FUNCTION:     Resample particles according to weights
%
% DESCRIPTION:  Follows systematic resampling method
%
% PARAMETERS:   particles: object handle, set of sample poses and weights
%
% RETURN:       particles: resampled particle set

factor = 1/particles.particle_count;
r =  (factor).*rand();
aux_particles = ParticleSet;
aux_particles.particle_count = particles.particle_count;
aux_particles.weights = zeros(particles.particle_count,1);
aux_particles.poses = zeros(particles.particle_count,3);
c = particles.weights(1);
i = 1;
for m = 1:particles.particle_count
    U = r + (m-1) * factor;
    while U >= c
        i = i + 1;
        c = c + particles.weights(i);
    end
    aux_particles.poses(m,:) = particles.poses(i,:);
    aux_particles.weights(m) = factor;
end
particles = aux_particles;

end