% export waypoints
clc;
simParam;
% Create waypoints_ros
% Assuming C is already defined and has the direction vectors in columns 4 to 6
% C = [..., direction_x, direction_y, direction_z, ...]

C_ordered = zeros(size(C_temp, 1), 3);
order_waypoints_c = order_waypoints_1(1, 2:(length(order_waypoints_1)-1));

for i = 1: length(order_waypoints_c)
    % waypointsOrdered(i, :) = waypoints(order_waypoints(1, i), :);
    C_ordered(i, :) = C_temp(order_waypoints_c(1, i), 4:6);
end

C_ordered = C_ordered(1:(length(C_ordered)-1), :);

% Extract the direction vectors
% directions_export = C(:, 4:6);
directions_export = C_ordered;

% directions_export = [0, -1, 0 ];

% Initialize a vector to store the headings
headings_export = zeros(size(directions_export, 1), 1);

% Compute the heading for each direction vector
for i = 1:size(directions_export, 1)
    direction_export = directions_export(i, :);
    % Heading is the angle in the x-y plane
    heading_export = -atan2(direction_export(2), direction_export(1)); % atan2(y, x)
    % Convert the heading from radians to degrees
    heading_export = rad2deg(heading_export);
    
    % Ensure heading is within the range [0, 360)
    % if heading_export < 0
    %     heading_export = heading_export + 360;
    % end
    
    % Store the computed heading
    headings_export(i) = heading_export;
end
headings_export = [TOAL_ros_heading; headings_export; TOAL_ros_heading];

% Display the headings
% disp(headings_export);

wp_temp = waypointsOrdered_1(2:(length(waypointsOrdered_1)-1), :);
wp_temp(:, 1) = -wp_temp(:, 1);
wp_temp(:, 3) = -wp_temp(:, 3);
wp_temp = wp_temp / 1000;

waypoints_ros = [TOAL_ros_pose; wp_temp; TOAL_ros_pose];


waypoints_ros = [waypoints_ros, headings_export]

% Choose the file path
file_path = 'waypoints_ros.txt';

% Open the file for writing
fileID = fopen(file_path, 'w');

% Write the waypoints to the file
[nrows, ncols] = size(waypoints_ros);
for row = 1:nrows
    fprintf(fileID, '%f\t', waypoints_ros(row, :));
    fprintf(fileID, '\n');
end

% Close the file
fclose(fileID);

disp(['waypoints_ros exported to ', file_path]);