function [points] = scans2freePoints(ground_truth, ranges)
% FUNCTION:     Samples the lines of ranges to create free points
%
% DESCRIPTION:  Along each range measurement line a set of points
%               is generated, which can be considered unobstructed
%               areas of the map.
%
% PARAMETERS:   ground_truth: true poses of robot for each scan
%               ranges: scans in array of distance measurements

points = [];
for i = 1:length(ground_truth)
    pose = ground_truth(i,:);
    points = [points; [ground_truth(i,1), ground_truth(i,2)]];
    for j = 1:length(ranges(i,:))
        theta = wrapToPi(pose(3) + pi * (j-1)/360 - pi/2);
        r = max(0.0, (ranges(i,j))-0.6).*rand(1,1);
        x = pose(1) + cos(theta) * r;
        y = pose(2) + sin(theta) * r;
        points = [points; [x, y]];
       
    end
end
end