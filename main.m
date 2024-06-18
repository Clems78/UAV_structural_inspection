% 2023-2024 Autonomous Vehicle Dynamics and Control MSc Individual Research Project:
% UAV Flight Planning Around Known Structures for Structural Inspections
% Sponsored by Marshall Futureworx
% Student: Clément Laguerre

vp_calculation = true;

if vp_calculation
close all;
clear;
clc;
vp_calculation = true;
end

% Import STL
file_name = 'STEP/cylinder_2.stl';

% Parameters viewpoints generation 
initial_guess = false;
opt = true; % Updating the input dataset or no

% Parameters TSP
tsp = true;
trajGeneration = false;
obj = 'comparison'; % 'duration' or 'battery' or 'comparison'

% Paremeters metrics 
overlap_calculation = true;

% Plotter parameters
plotter = true;
in_loop_plotter = true;
pause_time = 0.001;

% Loading simulation parameters
simParam;

% Viewpoints Generation
if vp_calculation
    processSTL;
end

% Path Generation using the Traveling Salesman Problem formulation 
if tsp
    switch obj 
        case 'duration'
            tic
            disp("TSP optimised for mission duration starts");
            TSP_s1;
            TSP_duration = toc;
            disp(['TSP optimised for mission duration done in ' num2str(TSP_duration) ' seconds']);
            [waypointsOrdered, nb_waypoints_ordered] = waypointsOrderingFun(Gsol, nb_waypoints, waypoints); 
            [path_length_1, alt_changes_1] = TSP_metrics(x_tsp, waypointsOrdered, output, waypoints);
            if (trajGeneration)
                [position, velocity, acceleration] = traj_generation(nb_waypoints_ordered, waypointsOrdered);
            end
        case 'battery'
            tic
            disp("TSP optimised for battery consumption starts");
            TSP_s2;
            TSP_bat_consumption = toc;
            disp(['TSP optimised for battery consumption done in ' num2str(TSP_bat_consumption) ' seconds']);
            [waypointsOrdered, nb_waypoints_ordered] = waypointsOrderingFun(Gsol_2, nb_waypoints, waypoints);
            [path_length_2, alt_changes_2] = TSP_metrics(x_tsp_2, waypointsOrdered, output_2, waypoints);
            if (trajGeneration)
                [position, velocity, acceleration] = traj_generation(nb_waypoints_ordered, waypointsOrdered);
            end            
        case 'comparison'
            % Mission duration
            tic
            disp("TSP optimised for mission duration starts");
            TSP_s1;
            TSP_duration = toc;
            disp(['TSP optimised for mission duration done in ' num2str(TSP_duration) ' seconds']);
            [waypointsOrdered_1, nb_waypoints_ordered] = waypointsOrderingFun(Gsol, nb_waypoints, waypoints);
            [path_length_1, alt_changes_1] = TSP_metrics(x_tsp, waypointsOrdered_1, output, waypoints);            
            if (trajGeneration)
                [position, velocity, acceleration] = traj_generation(nb_waypoints_ordered, waypointsOrdered_1);
            end
            % Battery consumption       
            tic
            disp("TSP optimised for battery consumption starts");
            TSP_s2;
            TSP_bat_consumption = toc;
            disp(['TSP optimised for battery consumption done in ' num2str(TSP_bat_consumption) ' seconds']);
            [waypointsOrdered_2, nb_waypoints_ordered] = waypointsOrderingFun(Gsol_2, nb_waypoints, waypoints);
            [path_length_2, alt_changes_2] = TSP_metrics(x_tsp_2, waypointsOrdered_2, output_2, waypoints);
            if (trajGeneration)
                [position_2, velocity_2, acceleration_2] = traj_generation(nb_waypoints_ordered, waypointsOrdered_2);
            end   
    end
end

% Overlap analysis
if (overlap_calculation)
    overlapCalculation;
end

% Plotting 
if (plotter)
    plotDetectionRange;
end
    