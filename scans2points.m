function points = scans2points(ground_truth, ranges)
%SCANS2POINTS Converts the scans at each pose into XY points

points = [];
for i = 1:length(ground_truth)
    pose = ground_truth(i,:);
    for j = 1:length(ranges(i,:))
        theta = wrapToPi(pose(3) + pi * (j-1)/360 - pi/2);
        x = pose(1) + cos(theta) * ranges(i,j);
        y = pose(2) + sin(theta) * ranges(i,j);
        points = [points; [x, y]];
    end
end
end