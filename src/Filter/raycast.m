function distance = raycast(model, pose, heading, ...
    raycast_occupancy_limit, raycast_sampling_interval, ...
    raycast_max_length, interpolation_flag)
% FUNCTION:     Raycast through map to find distance to object
%
% DESCRIPTION:  
%
% PARAMETERS:   model: object handle, classifier instance
%               pose: [x,y] array to begin casting from
%               heading: direction to cast in
%               raycast_occupancy_limit: required occupancy to terminate
%               raycast_sampling_interval: distance between each occupancy
%               check
%               raycast_max_length: maximum checked distance before
%               terminating
%
% RETURN:       particles: updated set with new weights, same poses

% Calculate iterable distance
dx = raycast_sampling_interval * cos(heading);
dy = raycast_sampling_interval * sin(heading);

distance = 0;
positions = [pose; pose(1) + dx, pose(2) + dy];
p_old = 0;

while distance < raycast_max_length
    p = double(model.classify(positions));
    p = p(:,2);
    index = find(p>raycast_occupancy_limit, 1,'first');
    if ~isempty(index)
        if interpolation_flag
            if index == 1
                m = (p(1) - p_old)/raycast_sampling_interval;
                c = p_old - distance * m;
                x = (raycast_occupancy_limit - c)/m;
            elseif index == 2
                m = (p(2) - p(1))/raycast_sampling_interval;
                c = p(1) - (distance + raycast_sampling_interval) * m;
                x = (raycast_occupancy_limit - c)/m;
            end
            distance = x;
            return
        else
            distance = distance + raycast_sampling_interval * index;
            return
        end
    else
        positions = positions + [2*dx, 2*dy; 2*dx, 2*dy];
        distance = distance + 2 * raycast_sampling_interval;
        p_old = p(2);
    end
end
distance = 100;
end