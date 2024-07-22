% 2023-2024 Autonomous Vehicle Dynamics and Control MSc Individual Research Project:
% UAV Flight Planning Around Known Structures for Structural Inspections
% Cranfield University
% Sponsored by Marshall Futureworx
% Student: Clément Laguerre

clc;
close all;

vp_calculation = false;

if vp_calculation
clear;
vp_calculation = true;
end

% Import STL
file_name = 'board.stl';

% Parameters viewpoints generation 
initial_guess = false;
opt = false; % Updating the input dataset or no

% Parameters TSP
tsp = true;
trajGeneration = false;
obj = 'comparison'; % 'duration' or 'battery' or 'comparison'
obj_s2 = "alt&path"; %"alt" or "alt&path"
opti_ratio = 2;

% Paremeters metrics 
overlap_calculation = true;
battery_consumption = true;

% Plotter parameters
plotter = true;
in_loop_plotter = false;
in_loop_printer = true;
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
            cprintf('Red', 'TSP optimised for mission duration starts\n');
            TSP_s1;
            TSP_duration = toc;
            cprintf('Red', 'TSP optimised for mission duration done in %f seconds\n', TSP_duration);
            [waypointsOrdered, nb_waypoints_ordered] = waypointsOrderingFun(Gsol, nb_waypoints, waypoints); 
            [path_length_1, alt_changes_1] = TSP_metrics(x_tsp, waypointsOrdered, output, waypoints);
            if (trajGeneration)
                [position, velocity, acceleration] = traj_generation(nb_waypoints_ordered, waypointsOrdered);
            end
            % Battery consumption calculation
            if battery_consumption
                E = energyConsumptionPath(waypointsOrdered, V); 
            end
        case 'battery'
            tic
            cprintf("Red", "TSP optimised for battery consumption starts\n");
            TSP_s2;
            TSP_bat_consumption = toc;
            cprintf('Red', 'TSP optimized for battery consumption done in %f seconds\n', TSP_bat_consumption);
            [waypointsOrdered, nb_waypoints_ordered] = waypointsOrderingFun(Gsol_2, nb_waypoints, waypoints);
            [path_length_2, alt_changes_2] = TSP_metrics(x_tsp_2, waypointsOrdered, output_2, waypoints);
            if (trajGeneration)
                [position, velocity, acceleration] = traj_generation(nb_waypoints_ordered, waypointsOrdered);
            end
            % Battery consumption calculation
            if battery_consumption
                E = energyConsumptionPath(waypointsOrdered, V); 
            end
        case 'comparison'
            % Mission duration
            tic
            cprintf('Red', 'TSP optimised for mission duration starts\n');
            TSP_s1;
            TSP_duration = toc;
            cprintf('Red', 'TSP optimised for mission duration done in %f seconds\n', TSP_duration);
            [waypointsOrdered_1, nb_waypoints_ordered] = waypointsOrderingFun(Gsol, nb_waypoints, waypoints);
            [path_length_1, alt_changes_1] = TSP_metrics(x_tsp, waypointsOrdered_1, output, waypoints);            
            if (trajGeneration)
                [position, velocity, acceleration] = traj_generation(nb_waypoints_ordered, waypointsOrdered_1);
            end
            % Battery consumption calculation
            if battery_consumption
                E_1 = energyConsumptionPath(waypointsOrdered_1, V); 
            end
            % Battery consumption       
            tic
            cprintf("Red", "TSP optimised for battery consumption starts\n");
            TSP_s2;
            TSP_bat_consumption = toc;
            cprintf('Red', 'TSP optimized for battery consumption done in %f seconds\n', TSP_bat_consumption);
            [waypointsOrdered_2, nb_waypoints_ordered] = waypointsOrderingFun(Gsol_2, nb_waypoints, waypoints);
            [path_length_2, alt_changes_2] = TSP_metrics(x_tsp_2, waypointsOrdered_2, output_2, waypoints);
            if (trajGeneration)
                [position_2, velocity_2, acceleration_2] = traj_generation(nb_waypoints_ordered, waypointsOrdered_2);
            end   
            % Battery consumption calculation
            if battery_consumption
                E_2 = energyConsumptionPath(waypointsOrdered_2, V); 
                if E_2 < E_1
                percentage = (E_1 - E_2) / E_1 * 100;
                cprintf('Blue', 'This path consumes %s%% less energy\n', num2str(percentage));
                end
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
    