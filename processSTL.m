% Processing the STL file
% Starting points
cprintf('Red', 'Clustering starts...\n');
tic;

% Import STL
[gm, fileformat, attributes, solidID] = stlread(file_name);

points_updated = zeros(size(gm.Points, 1), 3);

% Move structure
points_updated(:, 1) = gm.Points(:, 1) + x_transport;
points_updated(:, 2) = gm.Points(:, 2) + y_transport;
points_updated(:, 3) = gm.Points(:, 3) + z_transport;

gm = triangulation(gm.ConnectivityList, points_updated);

% Cluster number estimation + filtered ground nodes
clusterNbEstimation;   

if (initial_guess)
    k = k_est;
else 
    k = 1;
end

centroid = incenter(gm);
normal = faceNormal(gm);

Mtar = [centroid, normal];

% Define the threshold for the z-coordinate on the viewpoints generation
height = max(Mtar(:, 3)) - min(Mtar(:, 3));
% z_min_threshold_init = min(Mtar(:, 3)) + height * min_z_coeff - 100 ;
% z_max_threshold_init = max(Mtar(:, 3)) - height * max_z_coeff + 100;

% if (z_limit_vp_generation)
%     % Logical indexing to filter rows where the z-coordinate (3rd column) is higher than the threshold
%     Mtar_filtered = Mtar(Mtar(:, 3) > z_min_threshold_init & Mtar(:, 3) < z_max_threshold_init, :);
% else
%     Mtar_filtered = Mtar;
% end

% Initialise inspected samples
% inspected = zeros(size(Mtar_filtered, 1), 1);
inspected = [];

% height_updated = max(Mtar_filtered(:, 3)) - min(Mtar_filtered(:, 3));

diameter = 2 * rmaj_main;
nb_sections = ceil(height / 1000 / diameter);
disp([num2str(nb_sections), ' sections']);
z_min_threshold = zeros(nb_sections, 1);
z_max_threshold = zeros(nb_sections, 1);
vectors = [];
medoids = [];

vectors_temp = cell(1, nb_sections);
medoids_temp = cell(1, nb_sections);


for kk = 1:nb_sections
    if (kk == 1) 
        % z_min_threshold(kk) = z_min_threshold_init;
        z_min_threshold(kk) =  min(Mtar(:, 3)); %+ height * 0.1 ;
        z_max_threshold(kk) = z_min_threshold(kk) + height / nb_sections; %height_updated / nb_sections;
    else
        z_min_threshold(kk) = z_max_threshold(kk-1);
        z_max_threshold(kk) = z_min_threshold(kk) + height / nb_sections; % height_updated / nb_sections;
    end
end

% try
kk = 1;
full_size = 0;
Mtar_filtered = [];

for gg = 1:nb_sections
    if (z_limit_vp_generation)
        Mtar_filtered_temp = zeros(size(Mtar(Mtar(:, 3) >= z_min_threshold(gg) & Mtar(:, 3) <= z_max_threshold(gg), :), 1), size(Mtar(Mtar(:, 3) >= z_min_threshold(gg) & Mtar(:, 3) <= z_max_threshold(gg), :), 2), nb_sections);
        full_size = full_size + size(Mtar_filtered_temp, 1);
        Mtar_filtered_temp(:, :, gg) = Mtar(Mtar(:, 3) >= z_min_threshold(gg) & Mtar(:, 3) <= z_max_threshold(gg), :);
        
        Mtar_filtered = [Mtar_filtered; Mtar_filtered_temp(:, :, gg)];

        if gg < nb_sections
            Mtar_filtered_ns = zeros(size(Mtar(Mtar(:, 3) >= z_min_threshold(gg+1) & Mtar(:, 3) <= z_max_threshold(gg+1), :), 1), size(Mtar(Mtar(:, 3) >= z_min_threshold(gg) & Mtar(:, 3) <= z_max_threshold(gg), :), 2), nb_sections);
            Mtar_filtered_ns(:, :, gg) = Mtar(Mtar(:, 3) >= z_min_threshold(gg+1) & Mtar(:, 3) <= z_max_threshold(gg+1), :);
        end
    else
        Mtar_filtered_temp = Mtar;
    end    
    
    % Define the custom distance function with additional parameters using an anonymous function
    customDistFun = @(ZI, ZJ) distFun(ZI, ZJ, rmaj_main, rmaj_main, alpha_t); % with positionning uncertainty
    
    within_range = [length(k), 1];
    within_range_ns = [length(k), 1];
    
    all_inspected = false;
    
    % Create a vector to store the color of each triangle
    colors = zeros(size(gm.ConnectivityList, 1), 3); % Initialize with zeros for all triangles
    % colors = [1 0.6 0];
    % Map filtered indices back to the original structure
    filtered_indices = find(Mtar(:, 3) >= z_min_threshold(gg) & Mtar(:, 3) <= z_max_threshold(gg));
    if gg < nb_sections
        filtered_indices_ns = find(Mtar(:, 3) >= z_min_threshold(gg+1) & Mtar(:, 3) <= z_max_threshold(gg+1));
        inspected_ns = zeros(size(Mtar_filtered_ns(:, :, gg), 1), 1);
    end 
    
    if (gg == 1)
        inspected_current = zeros(size(Mtar_filtered_temp(:, :, gg), 1), 1);
    end

    % Print the surface
    trisurf(gm, 'FaceVertexCData', colors);
    
    % Initialise the samples as Mtar, then only cluster the samples that are
    % not already inspected
    Mtar_ni = Mtar_filtered_temp(:, :, gg);
    Mtar_ni_ns = Mtar_filtered_ns(:, :, gg);

    
    % Define the name of the file to save intermediate results
    checkpointFile = 'checkpoint.mat';

    % Initialize some data
    result = [];
    if (ns_not_inspected || (gg == 1))
        ii = 1;
       mm = 1;
    elseif (~ns_not_inspected)
            ii = 1;
    end

           % inspected
        while (~all_inspected)
        
            % Perform k-medoids clustering
            if (opt == true)
                if (ii == 1 && mm == 1)
                    [idx, C] = kmedoids(Mtar_ni(inspected_current(:,1) == false, :), k, 'Distance', customDistFun);
                    if in_loop_printer  
                        disp(['k = ', num2str(k)]);
                    end
                    ii = k + 1;
                    mm = mm + 1;
                    k = 1;
                else
                    % Mtar_ni(inspected(:,1) == false, :)
                    [idx, C(ii, :)] = kmedoids(Mtar_ni(inspected_current(:,1) == false, :), k, 'Distance', customDistFun);
                    if in_loop_printer
                        disp(['k = ', num2str(ii)]);
                    end
                    ii = ii + 1;
                    % Save the intermediate result every 10 iterations
                    if mod(ii, 10) == 0
                        save(checkpointFile, 'result', 'ii');
                        fprintf('Checkpoint saved at iteration %d\n', ii);
                    end
                end
            else
                [idx, C] = kmedoids(Mtar_ni, k, 'Distance', customDistFun);
                if in_loop_printer  
                    disp(['k = ', num2str(k)]);
                end    
            end
        
            % Parcourir tous les échantillons et les clusters
            for i = 1:size(Mtar_ni, 1)
                for j = 1:size(C, 1)
                    % Is the sample within inspection range ?
                    distance_cluster = sqrt((Mtar_ni(i, 1) - C(j, 1))^2 + (Mtar_ni(i, 2) - C(j, 2))^2 + (Mtar_ni(i, 3) - C(j, 3))^2)/1000;
                    within_range = distance_cluster < rmaj_main;
                    
                    % Is the sample inspected with an acceptable angle 
                    dot_product = dot(C(j, 4:6), Mtar_ni(i, 4:6));
                    mag_v1 = vecnorm(C(j, 4:6), 2);
                    mag_v2 = vecnorm(Mtar_ni(i, 4:6), 2);
                    angle = rad2deg(acos(dot_product / (mag_v1 * mag_v2)));
                    isWithinAngleThreshold = angle <= alpha_t;
                    
                    if within_range && isWithinAngleThreshold
                        inspected_current(i, 1) = true;
                        colors(filtered_indices(i), :) = [0, 0.8, 0];  % Update color for the original index                % disp("Sample inspected !");
                        % keep_rows = ~((1:size(Mtar_ni, 1)) == i);
                        % Mtar_ni = Mtar_ni(i ~= 1:size(Mtar_ni, 1), :);  % Logical indexing for removal
                        break;
                    else
                        inspected_current(i, 1) = false;
                        colors(filtered_indices(i), :) = [1, 0.6, 0];  % Update color for the original index                % disp("Sample not inspected !");
                    end             
                    % disp("continuing ?  ")
                end
            end
            if gg < nb_sections
                for i = 1:size(Mtar_ni_ns, 1)
                    for j = 1:size(C, 1)
                        % Is the sample within inspection range ?
                        distance_cluster_ns = sqrt((Mtar_ni_ns(i, 1) - C(j, 1))^2 + (Mtar_ni_ns(i, 2) - C(j, 2))^2 + (Mtar_ni_ns(i, 3) - C(j, 3))^2)/1000;
                        within_range_ns = distance_cluster_ns < rmaj_main;

                        % Is the sample inspected with an acceptable angle 
                        dot_product_ns = dot(C(j, 4:6), Mtar_ni_ns(i, 4:6));
                        mag_v1_ns = vecnorm(C(j, 4:6), 2);
                        mag_v2_ns = vecnorm(Mtar_ni_ns(i, 4:6), 2);
                        angle_ns = rad2deg(acos(dot_product_ns / (mag_v1_ns * mag_v2_ns)));
                        isWithinAngleThreshold_ns = angle_ns <= alpha_t;
                        if within_range_ns && isWithinAngleThreshold_ns
                            inspected_ns(i, 1) = true;
                            % disp("NEXT SECTION INSPECTED");
                            colors(filtered_indices_ns(i), :) = [0, 0.8, 1];  % Update color for the original index                % disp("Sample inspected !");
                            % keep_rows = ~((1:size(Mtar_ni, 1)) == i);
                            % Mtar_ni = Mtar_ni(i ~= 1:size(Mtar_ni, 1), :);  % Logical indexing for removal
                            break;
                        else
                            inspected_ns(i, 1) = false;
                            colors(filtered_indices_ns(i), :) = [1, 0.6, 1];
                        end
                    end     
                end
            end
        
            
            % Si tous les échantillons sont inspectés, sortir de la boucle
            if (inspected_current(:, 1) == true)
                all_inspected = true;
            else 
                all_inspected = false;
                if (opt == false)
                    k = k + 1;
                end
            end
            count = sum(inspected_current == 0);
            if in_loop_printer
                disp([num2str(count), ' samples out of ', num2str(length(Mtar_ni)), ' are still not inspected']);
            end
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
                    [ex, ey, ez] = ellipsoidPoints(cx, cy, cz, rmaj_main*1e3, rmaj_main*1e3, c_normal);
        
                    % Plot the ellipse
                    plot3(ex, ey, ez, 'r', 'LineWidth', 1);
                end
                hold off;
                pause(pause_time);
            end
        end

        % Mark as inspected the samples from the next section:
        inspected = [inspected; inspected_current];
        inspected_current = inspected_ns;


    % Compute the inspection position 
    % Create unitary vectors 
    % vectors(:, :, gg) = [];
    % medoids(:, :, gg) = [];
    vectors_temp{gg} = C(:, 4:6);
    medoids_temp{gg} = C(:, 1:3);
    vectors = [vectors; vectors_temp{gg}];
    medoids = [medoids; medoids_temp{gg}];


end

C = [medoids, vectors];

norm_vectors = vectors ./ sqrt(sum(vectors.^2, 2));

% Viewpoints generation
viewpoints = zeros(size(vectors, 1), 6);
viewpoints(:, 1:3) = medoids + d_insp_p * norm_vectors;
viewpoints(:, 4:6) = vectors;

% Waypoints generation
waypoints = viewpoints(:, 1:3) - camera_location;
waypoints = [start_point; waypoints];
C_temp = [[TOAL_ros_pose, start_direction]; C];
% C_temp = [C_temp; [TOAL_ros_pose, start_direction]];
nb_waypoints = length(waypoints);

clustering_duration = toc;

% Display that the clustering is done 
cprintf('Red', 'Clustering done in %f seconds\n\n', clustering_duration);
    
    % Final save after completing all iterations
    save('final_result.mat', 'result');
    fprintf('Final result saved\n');

% catch ME
%     % Save the current workspace if an error occurs
%     save('error_checkpoint.mat');
%     fprintf('Error occurred: %s\n', ME.message);
%     fprintf('Workspace saved to error_checkpoint.mat\n');
% end

