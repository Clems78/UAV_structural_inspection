% Coverage and overlap calculation:
rmaj_fixed_pp = false;
cprintf('Red', '\nTheory results\n');
[no_overlap, overlapped_twice, overlapped_thrice, overlapped_elmts, area_overlaped, area_not_cov] = overlapCalculation(nodes_list,ground_node, Mtar_filtered, C, Mtar_filtered(:, 1:3), rmaj_main, rmaj_fixed, Mtar_filtered(:, 4:6), alpha_t, points, area_structure, inspected);

cprintf('Red', '\nGT results\n');
[no_overlap_pp, overlapped_twice_pp, overlapped_thrice_pp, overlapped_elmts_pp, area_overlaped_pp, area_not_cov_pp] = overlapCalculation(nodes_list,ground_node, Mtar_filtered, C_pp, Mtar_filtered(:, 1:3), rmaj_p_2_pp, rmaj_fixed_pp, Mtar_filtered(:, 4:6), alpha_t, points, area_structure, inspected_pp);

% Average error of viewpoints position and orientation
waypoints_ros_pp = waypoints_ros;
waypoints_ros_pp(:, 3) = -waypoints_ros(:, 3);
dist_error = zeros(size(waypoints_ros_pp, 1), 1);
dist_error_medoids = zeros(size(C_pp, 1), 1);

% Reorder C 
C_ordered = zeros(size(C, 1), size(C, 2));

order_C_1 = order_waypoints_1';
order_C_1 = order_C_1(2:(size(order_C_1, 1)-1), size(order_C_1, 2));
order_C_1(:) = order_C_1(:) - 1;

for ii = 1:size(C, 1)
    C_ordered(ii, :) = C(order_C_1(ii), :);
end

% Rotation
waypoints_gt(:, 4) = heading_sto_pp;

for j = 1:size(waypoints_ros_pp, 1)
    if (waypoints_ros_pp(j, 4) < 0)
        waypoints_ros_pp(j, 4) = waypoints_ros_pp(j, 4) + 360;
    end
    if (waypoints_gt(j, 4) < 0)
        waypoints_gt(j, 4) = waypoints_gt(j, 4) + 360;
    end

    dist_error(j) = sqrt((waypoints_gt(j, 1) - waypoints_ros_pp(j, 1))^2 ...
        + (waypoints_gt(j, 2) - waypoints_ros_pp(j, 2))^2 ...
        + (waypoints_gt(j, 3) - waypoints_ros_pp(j, 3))^2 );
end

for i = 1:size(C_pp, 1)

    dist_error_medoids(i) = sqrt((C_ordered(i, 1) - C_pp(i, 1))^2 ...
    + (C_ordered(i, 2) - C_pp(i, 2))^2 ...
    + (C_ordered(i, 3) - C_pp(i, 3))^2 );
end


% Print results 
pose_error = abs(waypoints_gt - waypoints_ros_pp);
pose_error_2 = waypoints_ros_pp - waypoints_gt;
std(pose_error);
average_z_pose_error = mean(pose_error_2(:, 3));
max_error_alt = max(pose_error_2);
average_pose_error = mean(pose_error, 1);
mean_dist_error = mean(dist_error, 1);
mean_dist_error_medoids = nanmean(dist_error_medoids, 1);

cprintf('Red', '\nPose error \n');
disp(['Average x pose error: ', num2str(round(average_pose_error(1), 3)), ' m']);
disp(['Average y pose error: ', num2str(round(average_pose_error(2), 3)), ' m']);
disp(['Average z pose error: ', num2str(round(average_pose_error(3), 3)), ' m']);
disp(['Average heading error: ', num2str(round(average_pose_error(4), 1)), ' deg']);
disp(['Average distance error between waypoints: ', num2str(round(mean_dist_error, 3)), ' m']);
disp(['Average distance error between medoids: ', num2str(round(mean_dist_error_medoids/1000, 3)), ' m']);







% GSD 


% Not on target 
