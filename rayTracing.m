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
filePath = 'gt_pose_2024-07-30_01-09-16.csv';

% Read the CSV file
data = readtable(filePath);

% Extract numeric data, skipping the first row and first column (header and Waypoint)
waypoints_gt = table2array(data(:, 2:end));
waypoints_gt_pp = waypoints_gt(2:length(waypoints_gt)-1, :);

% Display the extracted data
disp(waypoints_gt_pp);
% 
% waypoints_gt = [
%     % start_point_pp;
%     % 0.844826, 1.279378, 2.111121, -0.016629, -0.010365, -0.00338, 0.999802;
%     % 1.969818, 1.368982, 2.130755, -0.000182, -0.024066, -0.005488, 0.999695;
%     2.168458, 1.312592, 3.386538, 0.003593, -0.001707, 0.009136, 0.99995;
%     2.011791, 1.375606, 4.939784, -0.002057, 0.000928, 0.013028, 0.999913;
%     2.102569, 1.317437, 6.117012, -0.001071, 0.000362, 0.014089, 0.9999;
%     1.117647, 1.271197, 6.243078, 0.001601, 0.015748, 0.01375, 0.99978;
%     -0.063358, 1.276549, 6.238632, 0.000431, 0.029301, 0.017029, 0.999425;
%     -1.451835, 1.296882, 6.292297, 0.004889, 0.024411, 0.014496, 0.999585;
%     -1.769334, 1.381201, 5.198371, -0.001925, 0.000394, 0.00917, 0.999956;
%     -0.621493, 1.345165, 4.703798, -0.002057, -0.023784, 0.010474, 0.99966;
%     0.638204, 1.336819, 4.764419, -0.00054, -0.027907, 0.010788, 0.999552;
%     0.8005, 1.312362, 3.584913, 0.000192, 0.004118, 0.005005, 0.999979;
%     -0.457775, 1.235915, 3.35901, 0.001847, 0.025586, -0.001705, 0.999669;
%     -1.757891, 1.238073, 3.580879, -0.000744, 0.028585, -0.005007, 0.999579;
%     -1.659833, 1.279484, 2.39667, 0.001551, 0.000071, -0.003131, 0.999994;
%     -0.411934, 1.349165, 2.345549, -0.003255, -0.029659, -0.004319, 0.999545;
%     % end_point_pp
% ];

% Variable initialisation 
C_pp = zeros(length(waypoints_gt_pp), 6);
rmaj_p_2_pp = zeros(length(waypoints_gt_pp), 1);
d_insp_p_pp = zeros(length(waypoints_gt_pp), 1);
G_pp = zeros(length(waypoints_gt_pp), 1);
on_target = zeros(length(waypoints_gt_pp), 1);
heading_sto =zeros(length(waypoints_gt_pp), 1);


for jj = 1:length(waypoints_gt_pp)

    q = [waypoints_gt_pp(jj, 7), waypoints_gt_pp(jj, 4), waypoints_gt_pp(jj, 5), waypoints_gt_pp(jj, 6)];
    
    % Normalize the quaternion (if not already normalized)
    q = q / norm(q);
    
    % Convert the quaternion to Euler angles
    euler = quat2eul(q);
    
    % Extract the heading (yaw) angle
    heading = euler(1);
    heading_transported = -heading + pi/2;
    
    % Optionally, convert to degrees if needed
    heading_deg = rad2deg(heading_transported);
    disp('Heading (yaw) in degrees:');
    disp(heading_deg);
    heading_sto(jj, 1) = heading_deg;
    
    % Update the Euler angles with the new heading
    euler(1) = heading_transported;
    
    % Convert the adjusted Euler angles back to a quaternion if needed
    adjusted_q = eul2quat(euler);
    
    % Define the camera position and orientation
    camera_location_gt = [waypoints_gt_pp(jj, 1)*1e3 + camera_location(1), waypoints_gt_pp(jj, 2) *1e3 + camera_location(2), waypoints_gt_pp(jj, 3) *1e3 + camera_location(3)]
    
    % Convert quaternion to rotation matrix
    R = quat2rotm(adjusted_q);
    
    % Define the camera direction (z-axis in the camera frame)
    camera_direction = transpose(R(:, 1)); % z-axis of the camera frame
    % camera_direction = [1 , 0 , 0]
    % Perform ray tracing to find intersection
    % Create a ray from the camera position in the camera direction
    rayOrigin = camera_location_gt;
    rayDirection = camera_direction;
    
    % Using Phased Array System Toolbox for ray tracing
    % Intersection with triangulated surface
    [intersect, t, u, v, xcoor] = rayTriangleIntersection(rayOrigin, rayDirection, gm.Points(gm.ConnectivityList(:, 1), :),gm.Points(gm.ConnectivityList(:, 2), :), gm.Points(gm.ConnectivityList(:, 3), :));
    
    % Find the first intersection point
    intersection_point = xcoor(intersect, :)

    if ~isempty(intersection_point)
        on_target(jj, 1) = 1;
        inspection_distance = zeros(size(intersection_point, 1), 1);
    
        % Calculate the distance from the camera to the intersection point
        for ii = 1:size(intersection_point, 1)
        inspection_distance(ii, :) = norm(intersection_point(ii,:) - camera_location_gt);
        end
        
        [d_insp_p_pp(jj), min_index] = min(inspection_distance) % Inspection distance
        
        C_pp(jj, :) = [intersection_point(min_index, :), -rayDirection];
        
        % Display the results
        % % disp('Intersection Point:');
        % disp(intersection_point);
        
        % disp('Inspection Distance:');
        % disp(d_insp_p_pp);
        
        % GSD calculation
        G_pp(jj) = d_insp_p_pp(jj) * sensor_height / (f * Ih);
        
        if (G_pp(jj) > G)
            disp("Invalid Ground Sampling Distance. Crack detection lowered");
        else 
            disp("Valid Ground Sampling Distance");
        end
        
        % Calculate width and height of the area covered
        W_p_pp = 2 * d_insp_p_pp(jj) * tan(horizontal_FOV_p / 2);
        H_p_pp = 2 * d_insp_p_pp(jj) * tan(vertical_FOV_p / 2);
        
        % Calculate the radius of the inspection area
        rmaj_p_2_pp(jj) = H_p_pp/2 / 1000;
    
        [d_insp_p_pp(jj), min_index] = min(inspection_distance); % Inspection distance
        
        C_pp(jj, :) = [intersection_point(min_index, :), -rayDirection];
        
        % Display the results
        % % disp('Intersection Point:');
        % disp(intersection_point);
        
        % disp('Inspection Distance:');
        % disp(d_insp_p_pp);
        
        % GSD calculation
        G_pp(jj) = d_insp_p_pp(jj) * sensor_height / (f * Ih);
        
        if (G_pp(jj) > G)
            disp("Invalid Ground Sampling Distance. Crack detection lowered");
        else 
            disp("Valid Ground Sampling Distance");
        end
        
        % Calculate width and height of the area covered
        W_p_pp = 2 * d_insp_p_pp(jj) * tan(horizontal_FOV_p / 2);
        H_p_pp = 2 * d_insp_p_pp(jj) * tan(vertical_FOV_p / 2);
        
        % Calculate the radius of the inspection area
        rmaj_p_2_pp(jj) = H_p_pp/2 / 1000;
    else
        C_pp(jj, :) = NaN;
    end
end

waypoints_gt = waypoints_gt(:, 1:4);
waypoints_gt(1, 4) = TOAL_ros_heading;
waypoints_gt(size(waypoints_gt, 1), 4) = TOAL_ros_heading;

waypoints_gt(2:(size(waypoints_gt, 1)-1), 4) = heading_sto;

Mtar_ni_pp = Mtar_filtered;

% Create a vector to store the color of each triangle
colors = zeros(size(gm.ConnectivityList, 1), 3); % Initialize with zeros for all triangles
% colors = [1 0.6 0];
        
% Print the surface
figure;
trisurf(gm, 'FaceVertexCData', colors);

% Initiaslize variables
within_range_pp = [length(C_pp), 1];
inspected_pp = zeros(size(Mtar_ni_pp, 1), 1);

% Evaluate 
for i = 1:size(Mtar_ni_pp, 1)
        for j = 1:size(C_pp, 1)
            % Is the sample within inspection range ?
            distance_cluster_pp = sqrt((centroid(i, 1) - C_pp(j, 1))^2 + (centroid(i, 2) - C_pp(j, 2))^2 + (centroid(i, 3) - C_pp(j, 3))^2)/1000;
            within_range_pp = distance_cluster_pp < rmaj_p_2_pp(j);
            
            % Is the sample inspected with an acceptable angle 
            dot_product = dot(C_pp(j, 4:6), normal(i, :));
            mag_v1 = vecnorm(C_pp(j, 4:6), 2);
            mag_v2 = vecnorm(normal(i, :), 2);
            angle = rad2deg(acos(dot_product / (mag_v1 * mag_v2)));
            isWithinAngleThreshold = angle <= alpha_t;
            
            if within_range_pp && isWithinAngleThreshold
                inspected_pp(i, 1) = true;
                colors(i, :) = [0, 0.8, 0];
                disp("Sample inspected !");
                % keep_rows = ~((1:size(Mtar_ni, 1)) == i);
                % Mtar_ni = Mtar_ni(i ~= 1:size(Mtar_ni, 1), :);  % Logical indexing for removal
                break;
            else
                inspected_pp(i, 1) = false;
                colors(i, :) = [1 0.6 0] ;
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
    trisurf(gm, 'FaceVertexCData', colors);
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
