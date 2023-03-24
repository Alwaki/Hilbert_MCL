function  visualizeHeatmap(model, xlim, ylim, resolution)
% FUNCTION:     Plot the probability distribution of occupancy
%
% DESCRIPTION:  By uniformly sampling the map area for its 
%               probability, we can create a heat map of the 
%               model through interpolation.
%
% PARAMETERS:   model: learned model of environment
%               xlim: map limits in horizontal direction
%               ylim: map limits in vertical direction
%               resolution: sampling interval in map

dx = xlim(2)-xlim(1);
dy = ylim(2)-ylim(1);
x_count = ceil(dx/resolution);
y_count = ceil(dy/resolution);
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
figure(2)
pcolor(sample_map)
colormap jet; shading interp, grid off
axis on
colorbar

% % Rescale plot
% xt = get(gca, 'XTick');
% set(gca, 'XTick', xt, 'XTickLabel', xt/20)
% yt = get(gca, 'YTick');
% set(gca, 'YTick', xt, 'YTickLabel', yt/10)