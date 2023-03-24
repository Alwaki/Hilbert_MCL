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
            % Assume pose of 1x3 structure
            obj.poses = repmat(initial_pose, particle_count, 1);
        end
        % Constructor global tracking
        function obj = initialize_unknownPose(obj, particle_count, xlim, ylim)
            obj.particle_count = particle_count;
            obj.weights = ones(particle_count,1).*1/particle_count;
            x = xlim(1) + (xlim(2)-xlim(1)).*rand(particle_count,1);
            y = ylim(1) + (ylim(2)-ylim(1)).*rand(particle_count,1);
            theta = wrapToPi((2*pi).*rand(particle_count,1));
            obj.poses = [x y theta];
        end
    end
end