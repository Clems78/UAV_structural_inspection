% Coverage and overlap calculation:
rmaj_fixed_pp = false;
[no_overlap_pp, overlapped_twice_pp, overlapped_thrice_pp, overlapped_elmts_pp, area_overlaped_pp, area_not_cov_pp] = overlapCalculation(nodes_list,ground_node, Mtar_ni_pp, C_pp, centroid, rmaj_p_2_pp, rmaj_fixed_pp, normal, alpha_t, points, area_structure, inspected_pp);

% Coverage and overlap of the origin calculation
[no_overlap, overlapped_twice, overlapped_thrice, overlapped_elmts, area_overlaped, area_not_cov] = overlapCalculation(nodes_list,ground_node, Mtar_filtered, C, centroid, rmaj_p_2, rmaj_fixed, normal, alpha_t, points, area_structure, inspected);

% Average error of viewpoints position and orientation
waypoints_ros_pp = waypoints_ros;
waypoints_ros_pp(:, 3) = -waypoints_ros(:, 3);
pose_error = abs(waypoints_gt - waypoints_ros_pp);

for i = 1:size(pose_error, 1)
    if (pose_error(i, 4) >= 360)
        pose_error(i, 4) = pose_error(i, 4) - 360;
    end
end

average_pose_error = mean(pose_error, 1);


% GSD 


% Not on target 
