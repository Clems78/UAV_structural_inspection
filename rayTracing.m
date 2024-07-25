%ray_tracing
clc;
% Define the triangulation structure
points_rt = gm.Points; % Coordinates of the nodes
connectivityList_rt = gm.ConnectivityList; % Indices forming each triangle

% -0.844826	1.279378	2.111121	-0.016629	-0.010365	-0.00338	0.999802


% Position of the drone when it took the picture in metres
x_pp = 0.621493; 
y_pp = 1.345165;
z_pp = 4.703798;
% wp

						


% Position of the drone and camera (no gimbal for now) when it took the picture
qx = -0.002057;
qy = -0.023784;
qz = 0.010474;
qw =  0.99966;

q = [qw, qx, qy, qz];

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

% Update the Euler angles with the new heading
euler(1) = heading_transported;

% Convert the adjusted Euler angles back to a quaternion if needed
adjusted_q = eul2quat(euler);

% Extract the pitch angle
pitch = euler(2);

% Display the result
disp('Pitch angle in radians:');
disp(pitch);

% Optionally, convert to degrees if needed
pitch_deg = rad2deg(pitch);
disp('Pitch angle in degrees:');
disp(pitch_deg);

% Display the result
% % Extract the roll angle
% % roll = euler(3);
% disp('Roll angle in radians:');
% % disp(roll);
% 
% % Optionally, convert to degrees if needed
% roll_deg = rad2deg(roll);
% disp('Roll angle in degrees:');
% disp(roll_deg);



% Define the camera position and orientation
camera_location_gt = [x_pp*1e3 + camera_location(1), y_pp*1e3 + camera_location(2), z_pp*1e3 + camera_location(3)];
% camera_location_gt = [1.9020*1e3, 1.1067*1e3  , 4.2612*1e3 ];

% camera_orientation = quaternion([qx, qy, qz, qw]); % Camera orientation as quaternion

% Convert quaternion to rotation matrix
R = quat2rotm(adjusted_q);


% Define the camera direction (z-axis in the camera frame)
camera_direction = transpose(R(:, 1)); % z-axis of the camera frame
% camera_direction = [0, 1, 0] % z-axis of the camera frame

% Perform ray tracing to find intersection
% Create a ray from the camera position in the camera direction
rayOrigin = camera_location_gt;
rayDirection = camera_direction;

% Using Phased Array System Toolbox for ray tracing
% Intersection with triangulated surface
[intersect, t, u, v, xcoor] = rayTriangleIntersection(rayOrigin, rayDirection, gm.Points(gm.ConnectivityList(:, 1), :),gm.Points(gm.ConnectivityList(:, 2), :), gm.Points(gm.ConnectivityList(:, 3), :));

% Find the first intersection point
intersection_point = xcoor(intersect, :);

inspection_distance = zeros(size(intersection_point, 1), 1);

% Calculate the distance from the camera to the intersection point
for ii = 1:size(intersection_point, 1)
inspection_distance(ii, :) = norm(intersection_point(ii,:) - camera_location_gt);
end

d_insp_p_pp = min(inspection_distance); % Inspection distance

% Display the results
disp('Intersection Point:');
disp(intersection_point);

disp('Inspection Distance:');
disp(d_insp_p_pp);


% GSD calculation
G_pp = d_insp_p_pp * sensor_height / (f * Ih);

if (G_pp > G_pp)
    disp("Invalid Ground Sampling Distance. Crack detection lowered");
else 
    disp("Valid Ground Sampling Distance");
end

% Calculate width and height of the area covered
W_p_pp = 2 * d_insp_p_pp * tan(horizontal_FOV_p / 2);
H_p_pp = 2 * d_insp_p_pp * tan(vertical_FOV_p / 2);

rmaj_p_2_pp = H_p_pp/2 / 1000;

Mtar_ni_pp = Mtar_filtered;

% Evaluate 
% for i = 1:size(Mtar_ni_pp, 1)
%         for j = 1:size(C, 1)
            % Is the sample within inspection range ?
            distance_cluster = sqrt((centroid(i, 1) - C(j, 1))^2 + (centroid(i, 2) - C(j, 2))^2 + (centroid(i, 3) - C(j, 3))^2)/1000;
            within_range = distance_cluster < rmaj_p_2;
            
            % Is the sample inspected with an acceptable angle 
            dot_product = dot(C(j, 4:6), normal(i, :));
            mag_v1 = vecnorm(C(j, 4:6), 2);
            mag_v2 = vecnorm(normal(i, :), 2);
            angle = rad2deg(acos(dot_product / (mag_v1 * mag_v2)));
            isWithinAngleThreshold = angle <= alpha_t;
            
            if within_range && isWithinAngleThreshold
                inspected(i, 1) = true;
                colors(i, :) = [0, 0.8, 0];
                % disp("Sample inspected !");
                % keep_rows = ~((1:size(Mtar_ni, 1)) == i);
                % Mtar_ni = Mtar_ni(i ~= 1:size(Mtar_ni, 1), :);  % Logical indexing for removal
                break;
            else
                inspected(i, 1) = false;
                colors(i, :) = [1 0.6 0] ;
                % disp("Sample not inspected !");
            end             
            % disp("continuing ?  ")
    %     end
    % end



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
