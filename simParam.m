% Simulation Parameters
% rmaj = 0.48
% rmin = 0.048 
% ==> Best results of clusters positionning somehow !

% Camera settings
sensor_width = 35.7; % camera frame width (mm) 
sensor_height = 23.8; % camera frame height (mm)
Iw = 9504 ; % Width resoluwidwition (pixel)
Ih = 6336; % Height resolution (pixel)
f = 50; % Focal lenght (mm)
camera_location = [120 120 200]; % Distance from the center of the drone (mm)

% UAV specification
CAuav = 1; % sphere radius for collision avoidance (m)
start_point = [0 0 0];
V = 3.5; % Speed of the uav

% Structure pose 
x_transport = 0;
y_transport = 5000;
z_transport = 5000;

% Mission specifications
% waiting time at each viewpoints
% Fixed speed (temporary)
% Take-off speed

% Ellipsoid representing the uav error in positioning 
% Is equal to the threshold to reach a waypoint
a = 200; % semi-axes lenght a (mm) major axis // to surface
b = 200; % semi-axes lenght b (mm) minor axis // to surface
c = 200; % semi-axes lenght c (mm) axis perpendicular to surface

% Detection parameters
alpha_t = 5; % max allowable angle between the camera's optical axis and the surface normal (degree)
G = 0.3; % Ground Sampling Distance (mm/pixel)
d_insp = G * f * Ih / sensor_height; % inspection distance / camera size and resolution should be the shortest distance between height and width

% Calculate horizontal and vertical FOV in radians (if not available by
% manufacturer, equation is available)
% horizontal_FOV = 2 * atan(sensor_width / (2 * f));
% vertical_FOV = 2 * atan(sensor_height / (2 * f));

horizontal_FOV = deg2rad(39.3);
vertical_FOV = deg2rad(26.8);

% Calculate width and height of the area covered
W = 2 * d_insp * tan(horizontal_FOV / 2);
H = 2 * d_insp * tan(vertical_FOV / 2);

alpha = 0.8;
rmaj = alpha*H/2 / 1000;     % detection range  /1000 to convert in meter
ellipse_ratio = 0.1;
rmin = rmaj * ellipse_ratio;
s = pi() * rmaj * rmin;

delta_theta = 0; % Error between the actual orientation of the drone and the desired one
d_insp_p = (d_insp - c) * cos(deg2rad(delta_theta)); % updated inspection distance 
theta = rad2deg(atan(Ih/f)); % Half the fov
theta_p = theta - delta_theta; % new theta 
fov = 2 * theta_p;

horizontal_FOV_p = horizontal_FOV - deg2rad(delta_theta);
vertical_FOV_p = vertical_FOV - deg2rad(delta_theta); 

% Calculate width and height of the area covered
W_p = 2 * d_insp_p * tan(horizontal_FOV_p / 2);
H_p = 2 * d_insp_p * tan(vertical_FOV_p / 2);

rmaj_p_2 = H_p/2 / 1000;
% rmaj_p_2 = 1;
s_p_2 = pi() * rmaj_p_2^2;  


% New s is define as an extremum problem 
% goal = find the rmaj
% step = find the position that minimizes xq = find the worst camera
% position pc. This position in on the arc define by the equation:

syms x y ;
eqn = x^2 / a^2 + (y - d_insp_p)^2 / c^2 == 1; 
xq = x + c * tan(deg2rad(theta_p)) * sqrt(1 - ((x^2)/(a^2))) - d_insp_p * tan(deg2rad(theta_p));

% Differentiate f_x to find the minimal value of xq 
df_x = diff(xq);
eqn_2 = df_x == 0;
x_min = double(solve(eqn_2, x));% the xq_min is obtained for x_min 
xq_min = double(subs(xq, x, x_min));

rmaj_p = d_insp_p * tan(deg2rad(theta_p)) - sqrt(a^2 + (c^2) * tan(deg2rad(theta_p))^2 );
s_p = pi() * rmaj_p^2;  



