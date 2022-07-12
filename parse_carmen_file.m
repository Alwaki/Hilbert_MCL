function [odom, ranges, ground_truth] = parse_carmen_file(logfile)
% Parses ranges and odometry separately from a carmen style file,
% as well as ground truth poses at each range reading.

data = readlines(logfile);
line_count = length(data);
odom = [];
ranges = [];
ground_truth = [];

for i = 1:line_count
    line = split(data(i));
    if line(1) == "ODOM"
        pose         = transpose(str2double(line(2:4)));
        odom         = [odom; pose];
    elseif line(1) == "FLASER"
        laser_count  = str2double(line(2));
        range        = str2double(line(3:laser_count+2));
        range        = transpose(range);
        ranges       = [ranges; range];
        gt           = transpose(str2double(line(laser_count+3:laser_count+5)));
        ground_truth = [ground_truth; gt];
    end

end

end