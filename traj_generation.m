function [position, velocity, acceleration] = traj_generation(nb_waypoints_ordered, waypointsOrdered)
% Trajectory generator
groundSpeed = zeros(nb_waypoints_ordered, 1);
groundSpeed(:,:) = 1;

% Define wait time of 1 second for all waypoints
waitTime = [0; 1; 0; 1; 0; 1; 0; 1]; % Multiply by 1 to ensure it's a numeric array

trajectory = waypointTrajectory(waypointsOrdered, GroundSpeed=groundSpeed);

t0 = trajectory.TimeOfArrival(1);
tf = trajectory.TimeOfArrival(end);
sampleTimes = linspace(t0,tf,100);

[position,~,velocity,acceleration,~] = lookupPose(trajectory,sampleTimes);
end