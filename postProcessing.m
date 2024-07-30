% Coverage and overlap calculation:
rmaj_fixed_pp = false;
[no_overlap_pp, overlapped_twice_pp, overlapped_thrice_pp, overlapped_elmts_pp, area_overlaped_pp, area_not_cov_pp] = overlapCalculation(nodes_list,ground_node, Mtar_ni_pp, C_pp, centroid, rmaj_p_2_pp, rmaj_fixed_pp, normal, alpha_t, points, area_structure, inspected_pp);

% Coverage and overlap of the origin calculation
[no_overlap, overlapped_twice, overlapped_thrice, overlapped_elmts, area_overlaped, area_not_cov] = overlapCalculation(nodes_list,ground_node, Mtar_filtered, C, centroid, rmaj_p_2, rmaj_fixed, normal, alpha_t, points, area_structure, inspected);

% Average error of viewpoints position and orientation
waypoints_ros_pp = waypoints_ros;
waypoints_ros_pp(:, 3) = -waypoints_ros(:, 3);

for j = 1:size(waypoints_ros_pp, 1)
    if (waypoints_ros_pp(j, 4) < 0)
        waypoints_ros_pp(j, 4) = waypoints_ros_pp(j, 4) + 360;
    end
    if (waypoints_gt(j, 4) < 0)
        waypoints_gt(j, 4) = waypoints_gt(j, 4) + 360;
    end
end

pose_error = abs(waypoints_gt - waypoints_ros_pp);



% for i = 1:size(pose_error, 1)
%     if (pose_error(i, 4) >= 360)
%         pose_error(i, 4) = pose_error(i, 4) - 360;
%     end
% end

average_pose_error = mean(pose_error, 1);

disp(['Average x pose error: ', num2str(round(average_pose_error(1), 3)), ' m']);
disp(['Average y pose error: ', num2str(round(average_pose_error(2), 3)), ' m']);
disp(['Average z pose error: ', num2str(round(average_pose_error(3), 3)), ' m']);
disp(['Average heading error: ', num2str(round(average_pose_error(4), 1)), ' deg']);





% GSD 


% Not on target 
