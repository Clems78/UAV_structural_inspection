   function tspExportFnc(wp_opt, filename, is3D, time_limit)
    % Check if wp_opt is a valid matrix
    if size(wp_opt, 2) < 2 || size(wp_opt, 2) > 3
        error('wp_opt must be an Nx2 or Nx3 matrix of coordinates.');
    end
    
    % Check if the is3D flag is provided; default to false (2D)
    if nargin < 3
        is3D = false;
    end
    
    % Get the number of waypoints
    num_waypoints = size(wp_opt, 1);
    
    % Define the base name for files
    [filepath, name, ~] = fileparts(filename);
    
    % Open the TSP file for writing
    tsp_filename = fullfile(filepath, [name, '.tsp']);
    fid = fopen(tsp_filename, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', tsp_filename);
    end
    
    % Write the header information
    fprintf(fid, 'NAME : %s\n', name);
    fprintf(fid, 'COMMENT : irp\n');
    fprintf(fid, 'TYPE : TSP\n');
    fprintf(fid, 'DIMENSION : %d\n', num_waypoints);
    if is3D
        fprintf(fid, 'EDGE_WEIGHT_TYPE : EUC_3D\n');
    else
        fprintf(fid, 'EDGE_WEIGHT_TYPE : EUC_2D\n');
    end
    fprintf(fid, 'NODE_COORD_SECTION\n');
    
    % Write the waypoints
    if is3D
        for i = 1:num_waypoints
            fprintf(fid, '%d %.5e %.5e %.5e\n', i, wp_opt(i, 1), wp_opt(i, 2), wp_opt(i, 3));
        end
    else
        for i = 1:num_waypoints
            fprintf(fid, '%d %.5e %.5e\n', i, wp_opt(i, 1), wp_opt(i, 2));
        end
    end
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
    if ~(time_limit == 0)
        fprintf(fid, 'TOTAL_TIME_LIMIT = %d\n', time_limit);
    end


    
    % Close the .par file
    fclose(fid);
    
    fprintf('Data successfully exported to %s and %s\n', tsp_filename, par_filename);
end

