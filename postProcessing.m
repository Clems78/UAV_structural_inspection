% Given quaternion components
q_x = -0.012667;
q_y = -0.00102;
q_z = 0.006364;
q_w = 0.999899;

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