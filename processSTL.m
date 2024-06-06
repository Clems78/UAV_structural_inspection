close all;
clear;
clc;

% Plot ?    
plotter = true;
in_loop_plotter = true;
pause_time = 0.001;
tsp = false;
trajGeneration = false;
opt = true;
initial_guess = false;

% Starting points
start_point = [-2000 -2000 0];

disp("Clustering starts");
tic;

simParam;

% Import STL
file_name = 'STEP/cylinder_2.stl';
[gm, fileformat, attributes, solidID] = stlread(file_name);

% Cluster number estimation
clusterNbEstimation;

if (initial_guess)
    k = k_est;
else 
    k = 1;
end

 
centroid = incenter(gm);
normal = faceNormal(gm);

Mtar = [centroid, normal];
Mtar_filtered = Mtar(any(Mtar(:, 3) ~= 0, 2), :);

% Define the custom distance function with additional parameters using an anonymous function
customDistFun = @(ZI, ZJ) distFun(ZI, ZJ, rmaj_p_2, rmaj_p_2, alpha_t); % with positionning uncertainty

within_range = [length(k), 1];
inspected = zeros(size(Mtar_filtered, 1), 1);
all_inspected = false;

% Create a vector to store the color of each triangle
colors = zeros(size(gm.ConnectivityList, 1), 3); % Initialize with zeros for all triangles
% colors = [1 0.6 0];
        
% Print the surface
trisurf(gm, 'FaceVertexCData', colors);

% Initialise the samples as Mtar, then only cluster the samples that are
% not already inspected
Mtar_ni = Mtar_filtered;
ii = 1;

while (~all_inspected)

    % Perform k-medoids clustering
    if (opt == true)
        if (ii == 1)
            [idx, C] = kmedoids(Mtar_ni, k, 'Distance', customDistFun);
            disp(['k = ', num2str(k)]);
            disp("test");
            ii = k + 1;
            k = 1;
        else
            [idx, C(ii, :)] = kmedoids(Mtar_ni(inspected(:,1) == false, :), k, 'Distance', customDistFun);
            disp(['k = ', num2str(ii)]);
            ii = ii + 1;
        end
    else
        [idx, C] = kmedoids(Mtar_ni, k, 'Distance', customDistFun);
        disp(['k = ', num2str(k)]);
    end

    % Parcourir tous les échantillons et les clusters
    for i = 1:size(Mtar_ni, 1)
        for j = 1:size(C, 1)
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
        end
    end
    
    % Si tous les échantillons sont inspectés, sortir de la boucle
    if (inspected(:, 1) == true)
        all_inspected = true;
    else 
        all_inspected = false;
        if (opt == false)
            k = k + 1;
        end
    end
    count = sum(inspected == 0);
    disp([num2str(count), ' samples out of ', num2str(length(Mtar_ni)), ' are still not inspected']);
    % disp('Cluster number increases');

    % Plot the surface
    if (in_loop_plotter)
        trisurf(gm, 'FaceVertexCData', colors);
        axis equal;
        hold on;
        scatter3(C(:,1), C(:,2), C(:,3), 50, "o", "r", 'filled');  % Plot medoids
        % Plot the ellipses around each centroid
        theta = linspace(0, 2*pi, 100);
        for i = 1:size(C, 1)
            % Centroid location
            cx = C(i, 1);
            cy = C(i, 2);
            cz = C(i, 3);

            % Directional information
            nx = C(i, 4);
            ny = C(i, 5);
            nz = C(i, 6);
            c_normal = [nx, ny, nz];

            % Ellipse in 3D space
            [ex, ey, ez] = ellipsoidPoints(cx, cy, cz, rmaj_p_2*1e3, rmaj_p_2*1e3, c_normal);

            % Plot the ellipse
            plot3(ex, ey, ez, 'r', 'LineWidth', 1);
        end
        hold off;
        pause(pause_time);
    end


end

% Compute the inspection position 
% Create unitary vectors 
vectors = C(:, 4:6);
medoids = C(:, 1:3);

norm_vectors = vectors ./ sqrt(sum(vectors.^2, 2));

% Viewpoints generation
viewpoints = medoids + d_insp_p * norm_vectors;

% Waypoints generation
waypoints = viewpoints + camera_location;
waypoints = [start_point; waypoints];
nb_waypoints = length(waypoints);

clustering_duration = toc;

% Display that the clustering is done 
disp(['Clustering done in ' num2str(clustering_duration) ' seconds']);

if (plotter)
    plotDetectionRange;
end

if (tsp)
    tic
    disp("TSP starts");
    TSP;
    TSP_duration = toc;

    % Display that the tsp is done 
    disp(['TSP done in ' num2str(TSP_duration) ' seconds']);
end

if (trajGeneration)
    trajectoryGeneration;
end

% Overlap analysis
overlapCalculation;

    