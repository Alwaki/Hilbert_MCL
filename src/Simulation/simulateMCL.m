function [error_pos, error_rot, stdev_pos, stdev_rot, mean_time, success_factor] = ...
    simulateMCL(logfile, full_angles, model, ...
    params, ground_truth, iteration)
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
if params.tracking_type
    particles = particles.initialize_unknownPose(params.global_particle_count,...
        params.xlim, params.ylim);
else
    particles = particles.initialize_knownPose(params.tracking_particle_count, ...
        params.waypoints(1,:));
end

% Create heatmap for plotting if applicable
if params.plotting_flag
    dx = params.xlim(2)-params.xlim(1);
    dy = params.ylim(2)-params.ylim(1);
    x_count = ceil(dx/params.resolution);
    y_count = ceil(dy/params.resolution);
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

% Default returns
error_pos = 0;
error_rot = 0;
stdev_pos = 0;
stdev_rot = 0;
mean_time = 0;
success_factor = 0;

% Split data in lines for sequential reading
data = readlines(logfile);
line_count = length(data);
prev_odom = [];

% Create statistics containers
euc_error = [];
heading_error = [];
times = [];
k = 1;

fprintf("Simulating run with id %d...\n", iteration)
% Loop through lines of data
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
                params.alpha);
            prev_odom = odom;
        end

    % Observation update
    elseif line(1) == "FLASER"
        laser_count   = str2double(line(2));
        ranges        = str2double(line(3:laser_count+2));
        ranges        = transpose(ranges);
        % Cutoff range measurements if needed
        if params.sensor_cutoff_flag == 1
            rangesToDelete = ranges > params.sensor_range_cutoff;
            ranges(rangesToDelete) = [];
        end
        % Inflate measurement noise if needed
        if params.inflate_noise == 1
            ranges = ranges + normrnd(0, params.sensor_range_noise, 1, length(ranges));
        end
        % Update belief
        [particles, time]    = observationUpdate(particles, ranges, full_angles(k,:),...
            model, params);

        % Generate error statistics
        [~,id] = max(particles.weights);
        euclidean = hypot(particles.poses(id,1)-ground_truth(k,1), ...
            particles.poses(id,2)-ground_truth(k,2));
        heading = abs(particles.poses(id,3)-ground_truth(k,3));
        k = k + 1;

        % Divergence/Convergence check
        if euclidean > 1.0 && params.tracking_type == 0
            fprintf("Tracking simulation with id %d diverged.\n", iteration)
            break
        elseif euclidean < 1.0 && params.tracking_type == 1
            fprintf("Global localization simulation with id %d converged.\n", iteration)
            break
        elseif k > 5 && params.tracking_type == 1
            fprintf("Global localization simulation with id %d did not converge.\n", iteration)
            break
        end
        
        % Add error statistics
        euc_error = [euc_error, euclidean];
        heading_error = [heading_error, heading];
        times = [times time];

    end
    
    % Visualize particle filter and map
    if params.plotting_flag
        pcolor(sample_map)
        colormap jet; shading interp, grid off
        hold on
        scaling = 1/params.resolution;
    
        axis(scaling*[params.xlim(1) params.xlim(2) params.ylim(1) params.ylim(2)])
        hold on
        plot_dir(ground_truth(:,1).*scaling, ground_truth(:,2).*scaling);
        hold on
        scatter(particles.poses(:,1).*scaling, ...
            particles.poses(:,2).*scaling, 20, "magenta", 'filled')
        scatter(particles.poses(id,1).*scaling, ...
            particles.poses(id,2).*scaling, 30, 'white', 'filled')
        hold off
        drawnow
    end
end

if(length(euc_error) == length(ground_truth))
        error_pos = rms(euc_error);
        error_rot = rms(heading_error);
        stdev_pos = std(euc_error);
        stdev_rot = std(heading_error);
        mean_time = mean(times);
        success_factor = 1/params.nbr_simulation_iterations;
        fprintf("Tracking simulation with id %d finished, adding data.\n", iteration)
end

end