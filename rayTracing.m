%ray_tracing
clc;
simParam;
% Define the triangulation structure
points_rt = gm.Points; % Coordinates of the nodes
connectivityList_rt = gm.ConnectivityList; % Indices forming each triangle

% -0.844826	1.279378	2.111121	-0.016629	-0.010365	-0.00338	0.999802

% Position of the drone when it took the picture in metres
% x_pp = 0.621493; 
% y_pp = 1.345165;
% z_pp = 4.703798;
% wp
start_point_pp = [0, 0, 0, 0, 0, 0, 1];
end_point_pp = [0, 0, 0, 0, 0, 0, 1];

% Define the path to the CSV file
filePath = 'gt_pose_2024-08-07_00-52-01.csv';

% Read the CSV file
data = readtable(filePath);

% Extract numeric data, skipping the first row and first column (header and Waypoint)
waypoints_gt = table2array(data(:, 2:end));
waypoints_gt_pp = waypoints_gt(2:length(waypoints_gt)-1, :);

waypoints_gt_pp(:, 1) = -waypoints_gt_pp(:, 1);

% Display the extracted data
disp(waypoints_gt_pp);

% Variable initialisation 
C_pp = zeros(length(waypoints_gt_pp), 6);
rmaj_p_2_pp = zeros(length(waypoints_gt_pp), 1);
d_insp_p_pp = zeros(length(waypoints_gt_pp), 1);
G_pp = zeros(length(waypoints_gt_pp), 1);
on_target = zeros(length(waypoints_gt_pp), 1);
heading_sto =zeros(length(waypoints_gt_pp), 1);
rayDirectionSto = zeros(length(waypoints_gt_pp), 3);


for jj = 1:length(waypoints_gt_pp)

    q = [-waypoints_gt_pp(jj, 7), -waypoints_gt_pp(jj, 4), waypoints_gt_pp(jj, 5), waypoints_gt_pp(jj, 6)];
    
    % Normalize the quaternion (if not already normalized)
    q = q / norm(q);
    
    % Convert the quaternion to Euler angles
    euler = quat2eul(q);
    
    % Extract the heading (yaw) angle
    heading = euler(1);

    heading_transported = -heading + pi/2;
    % heading_transported = -heading + pi/2;

    
    % Optionally, convert to degrees if needed
    heading_deg = rad2deg(heading_transported);
    % disp('Heading (yaw) in degrees:');
    % disp(heading_deg);
    heading_sto(jj, 1) = heading_deg;
    
    % Update the Euler angles with the new heading
    euler(1) = heading_transported;
    
    % Convert the adjusted Euler angles back to a quaternion if needed
    adjusted_q = eul2quat(euler);
    
    % Define the camera position and orientation
    camera_location_gt = [waypoints_gt_pp(jj, 1)*1e3 + camera_location(1), waypoints_gt_pp(jj, 2) *1e3 + camera_location(2), waypoints_gt_pp(jj, 3) *1e3 + camera_location(3)];
    
    % Convert quaternion to rotation matrix
    R = quat2rotm(adjusted_q);
    
    % Define the camera direction (z-axis in the camera frame)
    camera_direction = transpose(R(:, 1)); % z-axis of the camera frame
    % camera_direction = [1 , 0 , 0]
    % Perform ray tracing to find intersection
    % Create a ray from the camera position in the camera direction
    rayOrigin = camera_location_gt;
    rayDirection = camera_direction;
    rayDirectionSto(jj, :) = camera_direction;

    % Using Phased Array System Toolbox for ray tracing
    % Intersection with triangulated surface
    [intersect, t, u, v, xcoor] = rayTriangleIntersection(rayOrigin, rayDirection, gm.Points(gm.ConnectivityList(:, 1), :),gm.Points(gm.ConnectivityList(:, 2), :), gm.Points(gm.ConnectivityList(:, 3), :));
    
    % Find the first intersection point
    intersection_point = xcoor(intersect, :);

    if ~isempty(intersection_point)
        on_target(jj, 1) = 1;
        inspection_distance = zeros(size(intersection_point, 1), 1);
    
        % Calculate the distance from the camera to the intersection point
        for ii = 1:size(intersection_point, 1)
        inspection_distance(ii, :) = norm(intersection_point(ii,:) - camera_location_gt);
        end
        
        [d_insp_p_pp(jj), min_index] = min(inspection_distance); % Inspection distance
        
        C_pp(jj, :) = [intersection_point(min_index, :), -rayDirection];

        % GSD calculation
        G_pp(jj) = d_insp_p_pp(jj) * sensor_height / (f * Ih);

        % if (G_pp(jj) > GSD)
        %     disp("Invalid Ground Sampling Distance. Crack detection lowered");
        % else 
        %     disp("Valid Ground Sampling Distance");
        % end

        % Calculate width and height of the area covered
        W_p_pp = 2 * d_insp_p_pp(jj) * tan(horizontal_FOV_p / 2);
        H_p_pp = 2 * d_insp_p_pp(jj) * tan(vertical_FOV_p / 2);

        % Calculate the radius of the inspection area
        rmaj_p_2_pp(jj) = H_p_pp/2 / 1000;

    else
        C_pp(jj, :) = NaN;
    end
end

rayDirectionSto = [[0, 0, 1];rayDirectionSto; [0, 0, 1]];

heading_sto_pp = -heading_sto + 180;
heading_sto_pp =[TOAL_ros_heading; heading_sto_pp; TOAL_ros_heading ] 


waypoints_gt = waypoints_gt(:, 1:4);
waypoints_gt(1, 4) = TOAL_ros_heading;
waypoints_gt(size(waypoints_gt, 1), 4) = TOAL_ros_heading;

waypoints_gt(2:(size(waypoints_gt, 1)-1), 4) = heading_sto;

Mtar_ni_pp = Mtar_filtered;

% Create a vector to store the color of each triangle
colors_pp = zeros(size(gm.ConnectivityList, 1), 3); % Initialize with zeros for all triangles

filtered_indices_pp = find(Mtar(:, 3) > z_min_threshold & Mtar(:, 3) < z_max_threshold);

        
% Print the surface
figure;
trisurf(gm, 'FaceVertexCData', colors_pp);

% Initiaslize variables
within_range_pp = [length(C_pp), 1];
inspected_pp = zeros(size(Mtar_ni_pp, 1), 1);

% Evaluate 
for i = 1:size(Mtar_ni_pp, 1)
        for j = 1:size(C_pp, 1)
            % Is the sample within inspection range ?
            distance_cluster_pp = sqrt((Mtar_ni_pp(i, 1) - C_pp(j, 1))^2 + (Mtar_ni_pp(i, 2) - C_pp(j, 2))^2 + (Mtar_ni_pp(i, 3) - C_pp(j, 3))^2)/1000;
            within_range_pp = distance_cluster_pp < rmaj_p_2_pp(j);
            
            % Is the sample inspected with an acceptable angle 
            dot_product = dot(C_pp(j, 4:6), Mtar_ni_pp(i, 4:6));
            mag_v1 = vecnorm(C_pp(j, 4:6), 2);
            mag_v2 = vecnorm(Mtar_ni_pp(i, 4:6), 2);
            angle = rad2deg(acos(dot_product / (mag_v1 * mag_v2)));
            isWithinAngleThreshold = angle <= alpha_t;
            
            if within_range_pp && isWithinAngleThreshold
                inspected_pp(i, 1) = true;
                colors_pp(filtered_indices_pp(i), :) = [0, 0.8, 0];  % Update color for the original index                % disp("Sample inspected !");
                disp("Sample inspected !");
                % keep_rows = ~((1:size(Mtar_ni, 1)) == i);
                % Mtar_ni = Mtar_ni(i ~= 1:size(Mtar_ni, 1), :);  % Logical indexing for removal
                break;
            else
                inspected_pp(i, 1) = false;
                colors_pp(filtered_indices_pp(i), :) = [1, 0.6, 0];  % Update color for the original index                % disp("Sample inspected !");
                % disp("Sample not inspected !");
            end             
            % disp("continuing ?  ")
        end
        count_pp = sum(inspected_pp == 0);
    if in_loop_printer
        disp([num2str(count_pp), ' samples out of ', num2str(length(Mtar_ni_pp)), ' are still not inspected']);
    end
end
% Plot the surface
if (in_loop_plotter)
    trisurf(gm, 'FaceVertexCData', colors_pp);
    axis equal;
    hold on;
    scatter3(C_pp(:,1), C_pp(:,2), C_pp(:,3), 50, "o", "r", 'filled');  % Plot medoids
    % Plot the ellipses around each centroid
    theta = linspace(0, 2*pi, 100);
    for iii = 1:size(C_pp, 1)
        % Centroid location
        cx_pp = C_pp(iii, 1);
        cy_pp = C_pp(iii, 2);
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
    hold off;
    pause(0.001);
end

% figure(3);
% hold on
% scatter3(waypoints_gt(:,1)*1e3, waypoints_gt(:,2)*1e3, waypoints_gt(:,3)*1e3, 20, "o", "g", 'filled'); % Plot the viewpoints


function [intersect, t, u, v, xcoor] = rayTriangleIntersection(rayOrigin, rayDirection, V1, V2, V3)
    % Helper function for ray-triangle intersection
    % Möller–Trumbore intersection algorithm
    epsilon = 1e-6;
    intersect = false(size(V1, 1), 1);
    t = nan(size(V1, 1), 1);
    u = nan(size(V1, 1), 1);
    v = nan(size(V1, 1), 1);
    xcoor = nan(size(V1));
    
    for i = 1:size(V1, 1)
        edge1 = V2(i, :) - V1(i, :);
        edge2 = V3(i, :) - V1(i, :);
        h = cross(rayDirection, edge2);
        a = dot(edge1, h);
        if a > -epsilon && a < epsilon
            continue; % This ray is parallel to this triangle.
        end
        f = 1.0 / a;
        s = rayOrigin - V1(i, :);
        u(i) = f * dot(s, h);
        if u(i) < 0.0 || u(i) > 1.0
            continue;
        end
        q = cross(s, edge1);
        v(i) = f * dot(rayDirection, q);
        if v(i) < 0.0 || u(i) + v(i) > 1.0
            continue;
        end
        % At this stage we can compute t to find out where the intersection point is on the line.
        t(i) = f * dot(edge2, q);
        if t(i) > epsilon % ray intersection
            intersect(i) = true;
            xcoor(i, :) = rayOrigin + t(i) * rayDirection;
        end
    end
end
