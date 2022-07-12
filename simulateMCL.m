function [euc_error, heading_error] = simulateMCL(logfile, model, ...
    tracking_type, particle_count, xlim, ylim, alpha, gt, resolution, ...
    plotting_flag, raycast_occupancy_limit, raycast_sampling_interval, ...
    raycast_max_length, beam_nbr, sensor_variance, likelihood_model, ...
    point_mu, point_sigma)
% FUNCTION:     Attempts to perform localization with given data
%
% DESCRIPTION:  
%
% PARAMETERS:   logfile: CARMEN style data file
%               model: object handle, classifier instance
%               gt: full ground truth of path
%
%               see other parameters in parameter file
%
% RETURN:       ERROR STATISTICS

% Create particle filter
particles = ParticleSet;
if tracking_type
    particles = particles.initialize_unknownPose(particle_count*100, xlim, ylim);
else
    particles = particles.initialize_knownPose(particle_count,[2.5,2,0]);
end

if plotting_flag
    % Create heatmap for plotting
    dx = xlim(2)-xlim(1);
    dy = ylim(2)-ylim(1);
    x_count = ceil(dx/resolution);
    y_count = ceil(dy/resolution);
    sample_map = [];
    for i = 1:x_count
        sample_locations = zeros(y_count,2);
        for j = 1:y_count
            sample_locations(j,1) = (i-1)/x_count * dx;
            sample_locations(j,2) = (j-1)/y_count * dy;
        end
        result = double(model.classify(sample_locations));
        row = result(:,2);
        sample_map = [sample_map row];
    end
end

% Split data
data = readlines(logfile);
line_count = length(data);
prev_odom = [];

% Create statistics containers
euc_error = [];
heading_error = [];
k = 1;

% Loop through data
for i = 1:line_count
    line = split(data(i));
    % Motion update
    if line(1) == "ODOM"
        odom         = transpose(str2double(line(2:4)));
        [~,id] = max(particles.weights);
        if isempty(prev_odom)
            prev_odom = odom;
        else
            particles = motionUpdate(particles, prev_odom, odom, ...
                alpha);
            prev_odom = odom;
        end

    % Observation update
    elseif line(1) == "FLASER"
        laser_count  = str2double(line(2));
        range        = str2double(line(3:laser_count+2));
        range        = transpose(range);
        particles    = observationUpdate(particles, range, model, ...
            beam_nbr, raycast_occupancy_limit, raycast_sampling_interval, ...
            raycast_max_length, sensor_variance, likelihood_model, ...
            point_mu, point_sigma);
        % Error statistics collection
        [~,id] = max(particles.weights);
        euclidean = hypot(particles.poses(id,1)-gt(k,1), ...
            particles.poses(id,2)-gt(k,2));
        heading = abs(particles.poses(id,3)-gt(k,3));
        k = k + 1;
        if euclidean > 3.0 && tracking_type == 0
            disp("Run diverged, abandoning simulation.")
            break
        elseif euclidean < 1.0 && tracking_type == 1
            disp("Global localization converged.")
            break
        elseif k > 5 && tracking_type == 1
            disp("Global localization did not converge, abandoning simulation.")
            break
        end
        
        euc_error = [euc_error, euclidean];
        heading_error = [heading_error, heading];
        
      %  % TEMPORARY FOR VISUALIZATION OF LASER ENDPOINTS
%         measurements_used = 20;
%         measurements = range;
%         interval = floor(length(measurements(:))/measurements_used);
%         measurements = measurements(:,1:interval:end);
%         measurement_count = length(measurements);
%         angle_increment = pi / (measurement_count-1);
%         points = [];
%         for j = 1:measurement_count
%             headings = wrapToPi(particles.poses(:,3) + (j-1) * angle_increment - (pi/2));
%             points_x = particles.poses(:,1) + cos(headings) .* measurements(j);
%             points_y = particles.poses(:,2) + sin(headings) .* measurements(j);
%             points = [points; [points_x, points_y]];
%         end
    
    end

    if plotting_flag
        % Visualize filter
        pcolor(sample_map)
        colormap jet; shading interp, grid off
        hold on
        scaling = 1/resolution;
    
        % % TEMPORARY FOR VISUALIZATION OF LASER ENDPOINTS
        % if ~isempty(points)
        %   scatter(points(:,1).*scaling, points(:,2).*scaling, 16, 'yellow', 'filled')
        % end
    
        axis(scaling*[xlim(1) xlim(2) ylim(1) ylim(2)])
        hold on
        plot_dir(gt(:,1).*scaling, gt(:,2).*scaling);
        hold on
        scatter(particles.poses(:,1).*scaling, ...
            particles.poses(:,2).*scaling, 20, "magenta", 'filled')
        scatter(particles.poses(id,1).*scaling, ...
            particles.poses(id,2).*scaling, 30, 'white', 'filled')
        hold off
        drawnow
        %pause(1)
    end
end



end