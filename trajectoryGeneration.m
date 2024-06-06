% Trajectory generator
% clc;

% Trajectory generator
groundSpeed = zeros(nb_waypoints, 1);
groundSpeed(:,:) = 1;

% Define wait time of 1 second for all waypoints
waitTime = [0; 1; 0; 1; 0; 1; 0; 1]; % Multiply by 1 to ensure it's a numeric array

trajectory = waypointTrajectory(waypointsOrdered, GroundSpeed=groundSpeed);

t0 = trajectory.TimeOfArrival(1);
tf = trajectory.TimeOfArrival(end);
sampleTimes = linspace(t0,tf,100);

[position,~,velocity,acceleration,~] = lookupPose(trajectory,sampleTimes);

hold on;

plot3(position(:,1),position(:,2),position(:,3))
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
title("Trajectory")

% figure()
% subplot(3,1,1)
% plot(sampleTimes,velocity(:,1));
% ylabel("v_x (m/s)")
% title("Velocity Profile")
% subplot(3,1,2)
% plot(sampleTimes,velocity(:,2));
% ylabel("v_y (m/s)")
% subplot(3,1,3)
% plot(sampleTimes,velocity(:,3));
% ylabel("v_z (m/s)")
% xlabel("Time (sec)")
% 
% figure()
% subplot(3,1,1)
% plot(sampleTimes,acceleration(:,1));
% axis padded
% ylabel("a_x (m/s^2)")
% title("Acceleration Profile")
% subplot(3,1,2)
% plot(sampleTimes,acceleration(:,2));
% ylabel("a_y (m/s^2)")
% axis padded
% subplot(3,1,3)
% plot(sampleTimes,acceleration(:,3));
% ylabel("a_z (m/s^2)")
% xlabel("Time (sec)")

