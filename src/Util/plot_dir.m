function [h0, h1] = plot_dir (x, y)
% Plotting euclidean variables with direction towards next element

magnitude = 0.5;
dt = length(x);

% Indices
v0 = 1:(dt-1);
v1 = v0 + 1;

% Tails
vx0 = x(v0, 1);
vy0 = y(v0, 1);

% Heads
vx1 = x(v1, 1);
vy1 = y(v1, 1);

% Differences
dx = (vx1 - vx0) * magnitude;
dy = (vy1 - vy0) * magnitude;

% Plotting
h0 = plot (x, y, '.-'); 
hold on;
h1 = quiver (vx0,vy0, dx, dy, 0, 'r'); 
grid on; 
hold off
axis equal
