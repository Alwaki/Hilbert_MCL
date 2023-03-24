function distance = raycast(model, pose, heading, ...
    params)
%Raycast iterates point by point through map to find distance to 
%       object, which is considered an object if it is above
%       a threshold of occupancy.


% Calculate iterable distance
dx = params.raycast_sampling_interval * cos(heading);
dy = params.raycast_sampling_interval * sin(heading);

distance = 0;
positions = [pose; pose(1) + dx, pose(2) + dy];
p_old = 0;

while distance < params.raycast_max_length
    p = double(model.classify(positions));
    p = p(:,2);
    index = find(p>params.raycast_occupancy_limit, 1,'first');
    if ~isempty(index)
        if index == 1
            m = (p(1) - p_old)/params.raycast_sampling_interval;
            c = p_old - distance * m;
            x = (params.raycast_occupancy_limit - c)/m;
        elseif index == 2
            m = (p(2) - p(1))/params.raycast_sampling_interval;
            c = p(1) - (distance + params.raycast_sampling_interval) * m;
            x = (params.raycast_occupancy_limit - c)/m;
        end
        distance = x;
        return
    else
        positions = positions + [2*dx, 2*dy; 2*dx, 2*dy];
        distance = distance + 2 * params.raycast_sampling_interval;
        p_old = p(2);
    end
end
distance = 100;
end