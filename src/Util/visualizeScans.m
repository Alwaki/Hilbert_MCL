function visualizeScans(freePoints, hitPoints, ground_truth)
% FUNCTION:     Plot the endpoints of range measurements, as well
%               as the unoccupied areas generated by freePoints. Lastly
%               The ground_truth is also plotted.
%
% DESCRIPTION:  Simply plots the given points as a scatter form, and the
%               ground_truth points as interpolated lines.
%
% PARAMETERS:   freePoints: array of x,y points
%               hitPoints: array of x,y points
%               ground_truth: array of x,y points


% Plot points
figure(3)
scatter(hitPoints(:,1),hitPoints(:,2))
hold on
scatter(freePoints(:,1),freePoints(:,2))
legend("Occupied","Free")
%hold on
%plot(ground_truth(:,1),ground_truth(:,2), 'w-', 'LineWidth',2)
hold off