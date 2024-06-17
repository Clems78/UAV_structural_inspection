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