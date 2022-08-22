classdef ParticleSet
    %UNTITLED7 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        poses;
        weights;
        particle_count;
    end

    methods
        function obj = initialize_knownPose(obj, particle_count, initial_pose)
            %UNTITLED7 Construct an instance of this class
            %   Detailed explanation goes here
            obj.particle_count = particle_count;
            obj.weights = ones(particle_count,1).*1/particle_count;
            % Assume pose of 1x3 structure
            obj.poses = repmat(initial_pose, particle_count, 1);
        end

        function obj = initialize_unknownPose(obj, particle_count, xlim, ylim)
            %UNTITLED7 Construct an instance of this class
            %   Detailed explanation goes here
            obj.particle_count = particle_count;
            obj.weights = ones(particle_count,1).*1/particle_count;
            x = xlim(1) + (xlim(2)-xlim(1)).*rand(particle_count,1);
            y = ylim(1) + (ylim(2)-ylim(1)).*rand(particle_count,1);
            theta = wrapToPi((2*pi).*rand(particle_count,1));
            obj.poses = [x y theta];
        end
    end
end