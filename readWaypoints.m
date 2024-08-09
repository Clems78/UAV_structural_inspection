%reads waypoints_ros.txt file
% Define the file path
file_path = 'waypoints_ros_1_500.txt';

% Open the file for reading
file_id = fopen(file_path, 'r');

% Check if the file was opened successfully
if file_id == -1
    error('Cannot open file: %s', file_path);
end

% Read the data from the file
data = fscanf(file_id, '%f', [4, Inf]);

% Close the file
fclose(file_id);

% Transpose the data to match the desired format
waypoints_ros = data';

% Display the extracted data
disp(waypoints_ros);
