function wp_opt_lkh = tspSolReadFnc(wp_opt, filename)
    % Check if wp_opt is a valid matrix
    if size(wp_opt, 2) < 2 || size(wp_opt, 2) > 3
        error('wp_opt must be an Nx2 or Nx3 matrix of coordinates.');
    end

    % Open the TSP solution file for reading
    fid = fopen(filename, 'r');
    if fid == -1
        error('Cannot open file for reading: %s', filename);
    end
    
    % Initialize an empty array for storing the waypoint indices
    waypoint_indices = [];
    
    % Read the file line by line
    while ~feof(fid)
        line = fgetl(fid);
        
        % Skip the header lines and look for the TOUR_SECTION
        if contains(line, 'TOUR_SECTION')
            % Read the indices after 'TOUR_SECTION'
            while true
                index_line = fgetl(fid);
                if strcmp(index_line, '-1') || strcmp(index_line, 'EOF')
                    break;
                end
                waypoint_indices = [waypoint_indices; str2double(index_line)];
            end
        end
    end
    
    % Close the file
    fclose(fid);
    
    % Reorder the wp_opt array according to the indices in waypoint_indices
    wp_opt_lkh = wp_opt(waypoint_indices, :);
    
    fprintf('Waypoints successfully reordered according to the solution in %s\n', filename);
end
