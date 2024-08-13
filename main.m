% 2023-2024 Autonomous Vehicle Dynamics and Control MSc Individual Research Project:
% UAV Flight Planning Around Known Structures for Structural Inspections
% Cranfield University
% Sponsored by Marshall Futureworx
% Student: Clément Laguerre

clc;
close all;

vp_calculation = true;

if vp_calculation
clear;
vp_calculation = true;
end

z_limit_vp_generation = true; %limit the inspection on the z axis (tuning available in processSTL)
pareto_front_enabled = true;

% Import STL
% file_name = 'board.stl';
file_name = 'paper_test_piece.stl';
% file_name = 'cylinder_gz.stl';
% file_name = 'tower_nacelle.stl';

% Parameters viewpoints generation 
initial_guess = false;
opt = false; % Updating the input dataset or no
section = true;
ns_not_inspected = false;
min_z_coeff = 0;
max_z_coeff = 0;
y_min_distance = 1; %

% Parameters TSP
tsp = false;
trajGeneration = false;
obj = 'battery'; % 'duration' or 'battery' or 'comparison'
obj_s2 = "alt&path"; %"alt" or "alt&path"
if ~pareto_front_enabled
    opti_ratio = 0.45;
end

% Paremeters metrics 
overlap_calculation = true;
battery_consumption = true;

% Plotter parameters
plotter = false;
in_loop_plotter = true;
in_loop_printer = false;
pause_time = 0.001;

% pareto front calculation parameters

% Loading simulation parameters
simParamPaper;
% simParam;
rmaj_main = rmaj_p;
section_divider = 2*rmaj_main - (rmaj_main*2) * 0.3; % rmaj_main or 2 * rmaj_main

% Viewpoints Generation
if vp_calculation 
    if section
        processSTL;
    else
        processSTL_old;
    end
end

% Initialisation of tsp metrics variable in case of pareto front analysis


% Path Generation using the Traveling Salesman Problem formulation 
if tsp
    switch obj 
        case 'duration'
            tic
            cprintf('Red', 'TSP optimised for mission duration starts\n');
            % GA;
            % addpath('SA4TSP');
            % SA;
            TSP_s1;
            TSP_duration = toc;
            cprintf('Red', 'TSP optimised for mission duration done in %f seconds\n', TSP_duration);
            [waypointsOrdered, nb_waypoints_ordered, order_waypoints_1] = waypointsOrderingFun(Gsol, nb_waypoints, waypoints); 
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
            TSP_bat_consumption_duration = toc;
            cprintf('Red', 'TSP optimized for battery consumption done in %f seconds\n', TSP_bat_consumption_duration);
            [waypointsOrdered, nb_waypoints_ordered, order_waypoints_2] = waypointsOrderingFun(Gsol_2, nb_waypoints, waypoints);
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
            [waypointsOrdered_1, nb_waypoints_ordered, order_waypoints_1] = waypointsOrderingFun(Gsol, nb_waypoints, waypoints);
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
            [waypointsOrdered_2, nb_waypoints_ordered, order_waypoints_2] = waypointsOrderingFun(Gsol_2, nb_waypoints, waypoints);
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
                else
                percentage = (E_2 - E_1) / E_2 * 100;
                cprintf('Red', 'This path consumes %s%% more energy\n', num2str(percentage));    
                end
            end 
            
    end
end

% Overlap analysis
if (overlap_calculation)
    rmaj_fixed = true;
    [no_overlap, overlapped_twice, overlapped_thrice, overlapped_elmts, area_overlaped, area_not_cov] = overlapCalculation(gm.ConnectivityList,ground_node, Mtar_filtered, C, centroid, rmaj_main, rmaj_fixed, normal, alpha_t, points_updated, area_structure, inspected, filtered_indices_full);
end  

% Plotting 
if (plotter)
    plotDetectionRange;
end
    