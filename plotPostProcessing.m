%Plot gt viewpoints

figure(1);
axis equal;
hold on;
scatter3(C_pp(:,1), C_pp(:,2), C_pp(:,3), 50, "o", "y", 'filled');  % Plot medoids
    
camera_location_gt_plot = [waypoints_gt(:, 1)*1e3 + camera_location(1), waypoints_gt(:, 2) *1e3 + camera_location(2), waypoints_gt(:, 3) *1e3 + camera_location(3)];

% waypoints_gt_plot = waypoints_gt + camera_location;
camera_location_gt_plot(:, 1) = -camera_location_gt_plot(:, 1);

scatter3(camera_location_gt_plot(:,1), camera_location_gt_plot(:,2), camera_location_gt_plot(:,3), 20, "o", "g", 'filled'); % Plot the viewpoints

% Draw lines between each consecutive pair of waypoints
for i = 1:size(camera_location_gt_plot, 1) - 1
    plot3([camera_location_gt_plot(i,1) camera_location_gt_plot(i+1,1)], ...
          [camera_location_gt_plot(i,2) camera_location_gt_plot(i+1,2)], ...
          [camera_location_gt_plot(i,3) camera_location_gt_plot(i+1,3)], ...
          'k-', 'LineWidth', 1.5);
end

% Plot vectors at the waypoints locations
quiver3(camera_location_gt_plot(:,1), camera_location_gt_plot(:,2), camera_location_gt_plot(:,3), ...
        rayDirectionSto(:,1), rayDirectionSto(:,2), rayDirectionSto(:,3), ...
        'r', 'LineWidth', 3);
