% Coverage and overlap calculation:
rmaj_fixed_pp = false;
cprintf('Red', '\nTheory results\n');
[no_overlap, overlapped_twice, overlapped_thrice, overlapped_elmts, area_overlaped, area_not_cov] = overlapCalculation(nodes_list,ground_node, Mtar_filtered, C, centroid, rmaj_p_2, rmaj_fixed, normal, alpha_t, points, area_structure, inspected);

cprintf('Red', '\nGT results\n');
[no_overlap_pp, overlapped_twice_pp, overlapped_thrice_pp, overlapped_elmts_pp, area_overlaped_pp, area_not_cov_pp] = overlapCalculation(nodes_list,ground_node, Mtar_ni_pp, C_pp, centroid, rmaj_p_2_pp, rmaj_fixed_pp, normal, alpha_t, points, area_structure, inspected_pp);

% Average error of viewpoints position and orientation
waypoints_ros_pp = waypoints_ros;
waypoints_ros_pp(:, 3) = -waypoints_ros(:, 3);
dist_error = zeros(size(waypoints_ros_pp, 1), 1);
dist_error_medoids = zeros(size(C_pp, 1), 1);



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

    dist_error_medoids(i) = sqrt((C(i, 1) - C_pp(i, 1))^2 ...
    + (C(i, 2) - C_pp(i, 2))^2 ...
    + (C(i, 3) - C_pp(i, 3))^2 );
end


% Print results 
pose_error = abs(waypoints_gt - waypoints_ros_pp);
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
