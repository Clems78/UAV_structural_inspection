% Plotter

ellipsoid_plot = true;
plot_viewpoints = true;

% Plot the surface
trisurf(gm);
axis equal;
hold on;

% quiver3(centroid(:,1),centr
% oid(:,2),centroid(:,3), ...
%      normal(:,1),normal(:,2),normal(:,3),0.5,'color','m');

% Plot the clusters (3D plot for locational data)
% scatter3(Mtar(:,1), Mtar(:,2), Mtar(:,3), 10, idx, 'filled');
hold on;
scatter3(C(:,1), C(:,2), C(:,3), 50, "o", "r", 'filled');  % Plot medoids

if (plot_viewpoints)
    scatter3(viewpoints(:,1), viewpoints(:,2), viewpoints(:,3), 20, "o", "b", 'filled'); % Plot the viewpoints
    scatter3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 50, "o", "m", 'filled');
    scatter3(start_point(1), start_point(2), start_point(3), 50, "o", "y", 'filled'); % highlight the starting point
end
legend('Surface', 'Cluster medoids', 'Viewpoints', 'Waypoints', 'Start point');
title('K-Medoids Clustering with Custom Distance Function');
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;

if (ellipsoid_plot)
    % Plot the ellipses around each centroid
    theta = linspace(0, 2*pi, 100);
    for i = 1:size(C, 1)
        % Centroid location
        cx = C(i, 1);
        cy = C(i, 2);
        cz = C(i, 3);
    
        % Directional information
        nx = C(i, 4);
        ny = C(i, 5);
        nz = C(i, 6);
        c_normal = [nx, ny, nz];
    
        % Ellipse in 3D space
        [ex, ey, ez] = ellipsoidPoints(cx, cy, cz, rmaj_p_2*1e3, rmaj_p_2*1e3, c_normal);
    
        % Plot the ellipse
        plot3(ex, ey, ez, 'r', 'LineWidth', 0.5);
    end
end
