function [path_length, alt_changes] = TSP_metrics(x_tsp, waypointsOrdered, output, waypoints)
    % Absolute gap
    disp(['Absolute gap: ', num2str(output.absolutegap)]);
  
    % Calculate path length
    dist = pdist(waypoints, 'euclidean');
    path_length = sum(dist(x_tsp == 1))/1e3;
    path_length = round(path_length, 2);
    disp(['Path length: ', num2str(path_length), ' m']);
    
    % Calculate the altitude change
    alt_changes = 0;
    for i = 1:(length(waypointsOrdered)-1)
        alt_changes = alt_changes + abs(waypointsOrdered(i+1, 3) - waypointsOrdered(i, 3));
    end
    disp(['Overall altitude changes: ', num2str(round(alt_changes/1e3, 2)), ' m']);
end