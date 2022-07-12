function distance = raycast(model, pose, heading, ...
    raycast_occupancy_limit, raycast_sampling_interval, ...
    raycast_max_length)
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

while distance < raycast_max_length
    p = double(model.classify(positions));
    p = p(:,2);
    index = find(p>raycast_occupancy_limit, 1,'first');
    if ~isempty(index)
        distance = distance + raycast_sampling_interval * index;
        return
    else
        positions = positions + [2*dx, 2*dy; 2*dx, 2*dy];
        distance = distance + 2 * raycast_sampling_interval;
    end
end
distance = 100;
end