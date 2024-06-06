% Waypoints Ordering

edges = Gsol.Edges.EndNodes;

% Initialize the starting node
start_node = 1;

% Find the order of waypoints
num_edges = size(edges, 1);
order_waypoints = zeros(1, num_edges);
order_waypoints(1) = start_node;

% Traverse the graph to find the order of waypoints
current_node = start_node;
for i = 2:nb_waypoints +1
    [row, col] = find(edges == current_node, 1);
    if (col == 1)
        next_node = edges(row, 2);
    else
        next_node = edges(row, 1);
    end
    order_waypoints(i) = next_node;
    edges(row, :) = [];
    current_node = next_node;
end

% Display the order of waypoints
% disp('Order of waypoints:');
% disp(order_waypoints);

waypointsOrdered = zeros(nb_waypoints + 1, 3);

for i = 1:nb_waypoints+1
    waypointsOrdered(i, :) = waypoints(order_waypoints(1, i), :);
end

nb_waypoints = length(waypointsOrdered);