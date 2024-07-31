% Estimate the number of k 

% Calculate the area of the structure
nodes_list = gm.ConnectivityList;
points = gm.Points;
area_structure = 0;

% Remove the nodes that are on the ground (useful for the
% overlapCalculation script)
ground_node = length(nodes_list);

for i = 1:length(nodes_list)
    index1 = nodes_list(i, 1);
    index2 = nodes_list(i, 2);
    index3 = nodes_list(i, 3);

    nodes_matrix = [points(index1, :);
                    points(index2, :);
                    points(index3, :)];
    
    if ((points(index1, 3) == 0) && (points(index2, 3) == 0) && (points(index3, 3) == 0))
        ground_node(i) = true;
    else
        ground_node(i) = false;
    end

    if (ground_node(i) == false)
        area_structure = area_structure + getAreaTriangle(nodes_matrix/1000);
    end
end

% Get the number of clusters
k_est = round(area_structure / s_p) ;

if (k_est == 0)
    k_est = 1;
end

if (initial_guess)
    disp(['Estimated cluster number : ', num2str(k_est)]);
end

