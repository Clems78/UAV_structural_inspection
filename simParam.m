% Simulation Parameters

% Camera settings
horizontal_FOV = 39.3;
vertical_FOV = 26.8;

% If FOV not available by % manufacturer, equation is available to
% calculate it from the sensor size
% sensor_width = 35.7; % camera frame width (mm) 
sensor_height = 23.8; % camera frame height (mm)

Iw = 9504 ; % Width resoluwidwition (pixel)
Ih = 6336; % Height resolution (pixel)
f = 50; % Focal lenght (mm)
camera_location = [-30 120 229]; % Distance from the center of mass of the drone to the camera (mm)

% UAV specification
start_point = [0 0 0];
TOAL_ros_pose = [0, 0, -3];
TOAL_ros_heading = 90;
start_direction = [0, -1, 0];
V = 2; % Speed of the uav

% Structure pose 
x_transport = 0;
y_transport = 5000;
z_transport = 5000;
% z_transport = 70*1e3;

% Ellipsoid representing the uav error in positioning 
a = 200; % semi-axes lenght a (mm) major axis // to surface
b = 200; % semi-axes lenght b (mm) minor axis // to surface
c = 200; % semi-axes lenght c (mm) axis perpendicular to surface
delta_theta = 2; % Error between the actual orientation of the drone and the desired one

% Detection parameters
alpha_t = 20; % max allowable angle between the camera's optical axis and the surface normal (degree)
GSD = 0.3; % Ground Sampling Distance (mm/pixel)

%% Calculation

% Initial inspection distance 
d_insp = GSD * f * Ih / sensor_height; % inspection distance 
d_insp_p = (d_insp - c) * cos(deg2rad(delta_theta)); % updated inspection distance 

% Field Of View
horizontal_FOV_rad = deg2rad(39.3);
vertical_FOV_rad = deg2rad(26.8);

% Calculate horizontal and vertical FOV in radians (if not available by
% manufacturer, equation is available)
% horizontal_FOV_rad = 2 * atan(sensor_width / (2 * f));
% vertical_FOV_rad = 2 * atan(sensor_height / (2 * f));

theta_p = (rad2deg(vertical_FOV_rad))/2 - delta_theta;

rmaj_p = (d_insp_p/1000) * tan(deg2rad(theta_p)) - sqrt((a/1000)^2 + ((c/1000)^2) * (tan(deg2rad(theta_p)))^2 );
s_p = pi() * rmaj_p^2;  



