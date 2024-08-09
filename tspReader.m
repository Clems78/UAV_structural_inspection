% Specify the filename
filename = 'd1655.tsp';

% Open the file
fid = fopen(filename, 'r');

% Initialize an empty array to store coordinates
wp_opt = [];

% Loop through the file line by line
while ~feof(fid)
    % Get the current line
    line = fgetl(fid);
    
    % Check if the line contains node coordinates
    if contains(line, 'NODE_COORD_SECTION')
        % Skip the line with 'NODE_COORD_SECTION'
        continue;
    end
    
    % If we reach the end of the coordinate section
    if contains(line, 'EOF')
        break;
    end
    
    % Split the line by spaces
    tokens = str2num(line); %#ok<ST2NM>
    
    % Check if the line has at least three numbers (node index and two coordinates)
    if length(tokens) >= 3
        % Extract the coordinates (second and third numbers)
        coord = tokens(2:3);
        % Append the coordinates to wp_opt matrix
        wp_opt = [wp_opt; coord];
    end
end

% Close the file
fclose(fid);

% Display the result
disp('Extracted coordinates:');
disp(wp_opt);
