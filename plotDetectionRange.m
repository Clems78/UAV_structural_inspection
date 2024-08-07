% Plotter
ellipsoid_plotter = false;
viewpoints_plotter= true;
tsp_plotter = true;
traj_plotter = true;
overlap_plotter = false;
% tsp =true;

% Plot the surface
figure(2);
trisurf(gm);
axis equal;
hold on;
scatter3(C(:,1), C(:,2), C(:,3), 50, "o", "r", 'filled');  % Plot medoids


if (viewpoints_plotter)
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

if (ellipsoid_plotter)
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
        [ex, ey, ez] = ellipsoidPoints(cx, cy, cz, rmaj_main*1e3, rmaj_main*1e3, c_normal);
    
        % Plot the ellipse
        plot3(ex, ey, ez, 'r', 'LineWidth', 0.5);
    end
end

if strcmp(obj, 'comparison')
    hFig = gcf;
    hFigCopy = copyobj(hFig, 0);
    % Store the handle in the base workspace
    assignin('base', 'hFigCopy', hFigCopy);
end

% TSP ploter
if (tsp_plotter && tsp)
    switch obj
        case 'duration'
            figure(2);
            hold on;
            hGraph = plot(G,'XData',waypoints(:, 1),'YData',waypoints(:, 2), 'ZData', waypoints(:, 3), 'LineStyle','none','NodeLabel',{}, 'Marker','none', 'LineWidth',3, 'EdgeColor','b');
            highlight(hGraph,Gsol,'LineStyle','-');
            title('Inspection optimised for mission duration');
            drawnow;
            hold off;
        case 'battery'
            hGraph = plot(G_2,'XData',waypoints(:, 1),'YData',waypoints(:, 2), 'ZData', waypoints(:, 3), 'LineStyle','none','NodeLabel',{}, 'Marker','none', 'LineWidth',3, 'EdgeColor','b');
            highlight(hGraph,Gsol_2,'LineStyle','-');
            drawnow;
            title('Inspection optimised for battery consumption');
        case 'comparison'
            figure(2);
            hold on;
            hGraph = plot(G,'XData',waypoints(:, 1),'YData',waypoints(:, 2), 'ZData', waypoints(:, 3), 'LineStyle','none','NodeLabel',{}, 'Marker','none', 'LineWidth',3, 'EdgeColor','b');
            highlight(hGraph,Gsol,'LineStyle','-');
            title('Inspection optimised for mission duration');
            drawnow;
            hold off;
            hFigCopy = evalin('base', 'hFigCopy');
            figure(hFigCopy);
            hold on;
            hGraph_2 = plot(G_2,'XData',waypoints(:, 1),'YData',waypoints(:, 2), 'ZData', waypoints(:, 3), 'LineStyle','none','NodeLabel',{}, 'Marker','none', 'LineWidth',3, 'EdgeColor','b');
            highlight(hGraph_2,Gsol_2,'LineStyle','-');
            drawnow;
            title('Inspection optimised for battery consumption');
    end
end

% Traj plotter
if (traj_plotter && trajGeneration)
    switch obj
        case 'duration'
            plot3(position(:,1),position(:,2),position(:,3));
            title('Inspection optimised for mission duration + trajectory');
        case 'battery'
            plot3(position(:,1),position(:,2),position(:,3))
            title('Inspection optimised for battery consumption + trajectory');
        case 'comparison'
            % Duration
            figure(2);
            hold on;
            plot3(position(:,1),position(:,2),position(:,3))
            title('Inspection optimised for mission duration + trajectory');
            % Battery
            figure(hFigCopy)
            hold on;
            plot3(position_2(:,1),position_2(:,2),position_2(:,3))
            title('Inspection optimised for battery consumption + trajectory');
    end

end

if (overlap_plotter && overlap_calculation)
    % Category labels for the bar plot
    categories = {'No Overlap', 'Overlapped Elements', 'Overlapped Twice', 'Overlapped Thrice'};
    percentages = [no_overlap, overlapped_elmts, overlapped_twice, overlapped_thrice];
    % Create the bar plot
    figure(4);
    bar(percentages);
    % Set the x-tick labels using the category labels
    set(gca, 'XTick', 1:numel(categories), 'XTickLabel', categories);
    % Add a title and labels to the plot
    title('Overlap Analysis');
    xlabel('Overlap Status');
    ylabel('Percentage (%)');
    % Set the y-axis limits to [0, 100] to show percentages clearly
    ylim([0 100]);
end
