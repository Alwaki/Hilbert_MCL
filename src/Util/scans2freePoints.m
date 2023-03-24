function [points] = scans2freePoints(ground_truth, ranges, angles)
% FUNCTION:     Samples the lines of ranges to create free points
%
% DESCRIPTION:  Along each range measurement line a set of points
%               is generated, which can be considered unobstructed
%               areas of the map.
%
% PARAMETERS:   ground_truth: true poses of robot for each scan
%               ranges: scans in array of distance measurements

points = [];
for i = 1:length(ground_truth(:,1))
    pose = ground_truth(i,:);
    points = [points; [ground_truth(i,1), ground_truth(i,2)]];
    for j = 1:1:length(ranges(1,:))
        theta = wrapToPi(pose(3) + angles(i,j));
        r = min(ranges(i,j)-0.5, (ranges(i,j)).*rand(1,1));
        x = pose(1) + cos(theta) * r;
        y = pose(2) + sin(theta) * r;
        points = [points; [x, y]];
       
    end
end
end