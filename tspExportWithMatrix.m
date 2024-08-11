function tspExportWithMatrix(wp_opt, filename, comment, opti_ratio)
    % Check if wp_opt is a valid Nx2 or Nx3 matrix
    if size(wp_opt, 2) < 2 || size(wp_opt, 2) > 3
        error('wp_opt must be an Nx2 or Nx3 matrix of coordinates.');
    end
    
    % Get the number of waypoints
    num_waypoints = size(wp_opt, 1);
    

    altitudeFun = @(ZI, ZJ) altFun(ZI, ZJ);
    dist_eucli = pdist(wp_opt, 'euclidean');

    dist = (1 - opti_ratio) * pdist(wp_opt, altitudeFun) + opti_ratio * dist_eucli ;
    dist = squareform(dist);

    % Round distances to the nearest integer
    dist = round(dist);

    % Define the base name for files
    [filepath, name, ~] = fileparts(filename);
    
    % Open the TSP file for writing
    tsp_filename = fullfile(filepath, [name, '.tsp']);
    fid = fopen(tsp_filename, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', tsp_filename);
    end
    
    % Write the header information
    fprintf(fid, 'NAME: %s\n', name);
    fprintf(fid, 'TYPE: TSP\n');
    fprintf(fid, 'COMMENT: %s\n', comment);
    fprintf(fid, 'DIMENSION: %d\n', num_waypoints);
    fprintf(fid, 'EDGE_WEIGHT_TYPE: EXPLICIT\n');
    fprintf(fid, 'EDGE_WEIGHT_FORMAT: FULL_MATRIX\n');
    fprintf(fid, 'EDGE_WEIGHT_SECTION\n');
    
    % Write the distance matrix as integers
    for i = 1:num_waypoints
        fprintf(fid, '%d ', dist(i, :));
        fprintf(fid, '\n');
    end
    
    % Write EOF at the end of the file
    fprintf(fid, 'EOF\n');
    
    % Close the TSP file
    fclose(fid);

    % Create and write the .par file
    par_filename = fullfile(filepath, [name, '.par']);
    fid = fopen(par_filename, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', par_filename);
    end
    
    % Write the parameter information
    fprintf(fid, 'PROBLEM_FILE = %s\n', [name, '.tsp']);
    % fprintf(fid, 'OPTIMUM = 90000\n');
    % fprintf(fid, 'MOVE_TYPE = 5\n');
    % fprintf(fid, 'PATCHING_C = 3\n');
    % fprintf(fid, 'PATCHING_A = 2\n');
    % fprintf(fid, 'RUNS = 10\n');
    fprintf(fid, 'OUTPUT_TOUR_FILE = wp_opt_sol.tsp\n');

    % Close the .par file
    fclose(fid);
    
    fprintf('Data successfully exported to %s and %s\n', tsp_filename, par_filename);
    
end
