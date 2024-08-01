%Plot gt viewpoints
plot_ellipse = false;

figure(2);
axis equal;
hold on;
scatter3(C_pp(:,1), C_pp(:,2), C_pp(:,3), 50, "o", "g", 'filled');  % Plot medoids
    
camera_location_gt_plot = [waypoints_gt(:, 1)*1e3 + camera_location(1), waypoints_gt(:, 2) *1e3 + camera_location(2), waypoints_gt(:, 3) *1e3 + camera_location(3)];

% waypoints_gt_plot = waypoints_gt + camera_location;
camera_location_gt_plot(:, 1) = -camera_location_gt_plot(:, 1);

scatter3(camera_location_gt_plot(:,1), camera_location_gt_plot(:,2), camera_location_gt_plot(:,3), 20, "o", "g", 'filled'); % Plot the viewpoints

% Draw lines between each consecutive pair of waypoints
for i = 1:size(camera_location_gt_plot, 1) - 1
    plot3([camera_location_gt_plot(i,1) camera_location_gt_plot(i+1,1)], ...
          [camera_location_gt_plot(i,2) camera_location_gt_plot(i+1,2)], ...
          [camera_location_gt_plot(i,3) camera_location_gt_plot(i+1,3)], ...
          'k-', 'LineWidth', 2);
end

% Plot vectors at the waypoints locations
quiver3(camera_location_gt_plot(:,1), camera_location_gt_plot(:,2), camera_location_gt_plot(:,3), ...
        rayDirectionSto(:,1), rayDirectionSto(:,2), rayDirectionSto(:,3), ...
        'g', 'LineWidth', 1.5);
if plot_ellipse
    % Plot the ellipses around each centroid
    theta = linspace(0, 2*pi, 100);
    for iii = 1:size(C_pp, 1)
        % Centroid location
        cx_pp = C_pp(iii, 1);
    s    cy_pp = C_pp(iii, 2);
        cz_pp = C_pp(iii, 3);

        % Directional information
        nx_pp = C_pp(iii, 4);
        ny_pp = C_pp(iii, 5);
        nz_pp = C_pp(iii, 6);
        c_normal_pp = [nx_pp, ny_pp, nz_pp];

        % Ellipse in 3D space
        [ex_pp, ey_pp, ez_pp] = ellipsoidPoints(cx_pp, cy_pp, cz_pp, rmaj_p_2_pp(iii)*1e3, rmaj_p_2_pp(iii)*1e3, c_normal_pp);

        % Plot the ellipse
        plot3(ex_pp, ey_pp, ez_pp, 'r', 'LineWidth', 1);
    end
end
