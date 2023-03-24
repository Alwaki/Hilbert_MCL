function [particles] = motionUpdate(particles, prev_odom, new_odom, alpha)
% FUNCTION:     Samples a new robot pose according to odometry motion model
%
% DESCRIPTION:  Updates the pose of each particle according to the change
%               in odometry measurement. This function updates all
%               particles at the same time to utilize matrix speedups with
%               Matlab.
%
% PARAMETERS:   particles: set of sample poses and weights
%               prev_odom: last odometry measurement
%               new_odom: incoming odometry measurement
%               alpha: motion noise parameters
%
% RETURN:       particles: updated set of sample poses with same weights

d_rot_1     = atan2(new_odom(2) - prev_odom(2), ...
              new_odom(1) - prev_odom(1)) - prev_odom(3);

d_trans     = hypot(prev_odom(1) - new_odom(1), ...
              prev_odom(2) - new_odom(2));

d_rot_2     = new_odom(3) - prev_odom(3) - d_rot_1;

d_rot_1_hat = d_rot_1 - normrnd(0, alpha(1) * abs(d_rot_1) + ...
                               alpha(2) * abs(d_trans), ...
                               particles.particle_count, 1);

d_trans_hat = d_trans - normrnd(0, alpha(3) * abs(d_trans) + ...
                               alpha(4) * abs(d_rot_1 + d_rot_2), ...
                               particles.particle_count, 1);

d_rot_2_hat = d_rot_2 - normrnd(0, alpha(1) * abs(d_rot_2) + ...
                               alpha(2) * abs(d_trans), ...
                               particles.particle_count, 1);

particles.poses(:,1) = particles.poses(:,1) + d_trans_hat .* ...
    cos(particles.poses(:,3) + d_rot_1_hat);

particles.poses(:,2) = particles.poses(:,2) + d_trans_hat .* ...
    sin(particles.poses(:,3) + d_rot_1_hat);

particles.poses(:,3) = wrapToPi(particles.poses(:,3) + ...
    d_rot_1_hat + d_rot_2_hat);

end