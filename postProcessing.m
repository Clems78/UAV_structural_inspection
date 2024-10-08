clc;
simParam;

% Given quaternion components
% q_x = -0.000132;
% q_y = 0.000398;
% q_z = -0.004731;
% q_w = 0.999899;
% -0.000132	0.000398	-0.004731	0.999989

q_x = -0.016629;
q_y = -0.010365;
q_z = -0.00338;
q_w =  0.999802;



% Position of the drone when it took the picture in metres
x_pp = 1.122386; 
y_pp = 1.123265;
z_pp = 1.133849;

% GT position of the camera 
camera_location_gt = [x_pp*1e3 + camera_location(1), y_pp*1e3 + camera_location(2), z_pp*1e3 + camera_location(3)];

% Create the quaternion
q = [q_w, q_x, q_y, q_z];  % MATLAB uses the format [w, x, y, z]

% Normalize the quaternion (if not already normalized)
q = q / norm(q);

% Convert the quaternion to Euler angles
euler = quat2eul(q);

% Extract the heading (yaw) angle
heading = euler(1);
heading_transported = -heading + pi/2;

% Display the result
disp('Heading (yaw) in radians:');
disp(heading_transported);

% Optionally, convert to degrees if needed
heading_deg = rad2deg(heading_transported);
disp('Heading (yaw) in degrees:');
disp(heading_deg);

% Extract the pitch angle
pitch = euler(2);

% Display the result
disp('Pitch angle in radians:');
disp(pitch);

% Optionally, convert to degrees if needed
pitch_deg = rad2deg(pitch);
disp('Pitch angle in degrees:');
disp(pitch_deg);

% Extract the roll angle
roll = euler(3);

% Display the result
disp('Roll angle in radians:');
disp(roll);

% Optionally, convert to degrees if needed
roll_deg = rad2deg(roll);
disp('Roll angle in degrees:');
disp(roll_deg);

% GT viewpoints
gt_viewpoints = [camera_location_gt, heading_deg];

% Calculate the distance to the surface = inspection distance 
% Is the GSD acceptable ? 
% Calculate the resulting inspection range
% for each triangle, is it within range and in front of the viewpoints with
% different ranges for all viewpoints
%==> Calculate the area of the triangles inspected vs not inspected =
%coverage percentage
% usuall overlap calculation