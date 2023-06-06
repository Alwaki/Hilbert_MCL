classdef ParticleSet
    % ParticleSet container for particles (samples)

    properties
        poses;
        weights;
        particle_count;
    end

    methods
        % Constructor local tracking
        function obj = initialize_knownPose(obj, particle_count, initial_pose)
            obj.particle_count = particle_count;
            obj.weights = ones(particle_count,1).*1/particle_count;
            obj.poses = repmat(initial_pose, particle_count, 1);
        end
        % Constructor global tracking
        function obj = initialize_unknownPose(obj, particle_count, ground_truth)
            obj.particle_count = particle_count;
            obj.weights = ones(particle_count,1).*1/particle_count;
            sampled_pos = datasample(ground_truth(:,1:2), particle_count, 1);
            perturbed_pos = sampled_pos + -1 + (1+1).*rand(particle_count,2);
            theta = wrapToPi((2*pi).*rand(particle_count,1));
            obj.poses = [perturbed_pos(:,1) perturbed_pos(:,2) theta];
        end
    end
end
