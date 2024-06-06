% Estimate the number of k 

% Calculate the area of the structure
nodes_list = gm.ConnectivityList;
points = gm.Points;
area_structure = 0;

for i = 1:length(nodes_list)
    index1 = nodes_list(i, 1);
    index2 = nodes_list(i, 2);
    index3 = nodes_list(i, 3);

    nodes_matrix = [points(index1, :);
                    points(index2, :);
                    points(index3, :)];
    
    area_structure = area_structure + getAreaTriangle(nodes_matrix/1000);
end

% Get the number of clusters
k_est = round(area_structure / s_p_2);

if (k_est == 0)
    k_est = 1;
end

if (initial_guess)
    disp(['Estimated cluster number : ', num2str(k_est)]);
end

function area_triangle = getAreaTriangle(nodes_coordinates)

% Calculate the side lengths
  s1 = norm(nodes_coordinates(2,:) - nodes_coordinates(1,:));
  s2 = norm(nodes_coordinates(3,:) - nodes_coordinates(1,:));
  s3 = norm(nodes_coordinates(3,:) - nodes_coordinates(2,:));

  % Calculate the semi-perimeter
  sp = (s1 + s2 + s3) / 2;

  % Calculate the area using Heron's formula
  area_triangle = sqrt(sp * (sp - s1) * (sp - s2) * (sp - s3));
end