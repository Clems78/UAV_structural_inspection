function [no_overlap, overlapped_twice, overlapped_thrice, overlapped_elmts, area_overlaped, area_not_cov, area_structure] = overlapCalculation(nodes_list,ground_node, Mtar_filtered, C, rmaj, rmaj_fixed, alpha_t, points, inspected, filtered_indices_full)
% Assess overlap
inspected_overlap = zeros(size(Mtar_filtered, 1), 1);
area_overlaped = 0;
area_not_cov = 0;
area_structure = 0;

centroid = Mtar_filtered(:, 1:3);
normal = Mtar_filtered(:, 4:6);

for i = 1:size(Mtar_filtered, 1)
    % disp(["Processing polygons #", num2str(i)]);
    for j = 1:size(C, 1)
        distance_cluster = sqrt((centroid(i, 1) - C(j, 1))^2 + (centroid(i, 2) - C(j, 2))^2 + (centroid(i, 3) - C(j, 3))^2)/1000;
        if rmaj_fixed
            within_range = distance_cluster < rmaj;
        else
            within_range = distance_cluster < rmaj(j);
        end
        
        % Is the sample inspected with an acceptable angle 
        dot_product = dot(C(j, 4:6), normal(i, :));
        mag_v1 = vecnorm(C(j, 4:6), 2);
        mag_v2 = vecnorm(normal(i, :), 2);
        angle = rad2deg(acos(dot_product / (mag_v1 * mag_v2)));
        isWithinAngleThreshold = angle <= alpha_t;
        
        if within_range && isWithinAngleThreshold
            inspected_overlap(i, 1) = inspected_overlap(i, 1) + 1;
        end  
    end

    % Area structure calculation
    index1_struct = nodes_list(filtered_indices_full(i), 1);
    index2_struct= nodes_list(filtered_indices_full(i), 2);
    index3_struct= nodes_list(filtered_indices_full(i), 3);
    
    nodes_matrix_struct = [points(index1_struct, :);
            points(index2_struct, :);
            points(index3_struct, :)];    
    area_structure = area_structure + getAreaTriangle(nodes_matrix_struct/1000);

    if (inspected_overlap(i, 1) >= 2)
        % Getting the area covered by the polygon
        index1_overlap = nodes_list(filtered_indices_full(i), 1);
        index2_overlap = nodes_list(filtered_indices_full(i), 2);
        index3_overlap = nodes_list(filtered_indices_full(i), 3);
    
        nodes_matrix_overlap = [points(index1_overlap, :);
                points(index2_overlap, :);
                points(index3_overlap, :)];
    
        area_overlaped = area_overlaped + getAreaTriangle(nodes_matrix_overlap/1000);
    end

    if (inspected_overlap(i, 1) == 0)
        index1_cov = nodes_list(filtered_indices_full(i), 1);
        index2_cov = nodes_list(filtered_indices_full(i), 2);
        index3_cov = nodes_list(filtered_indices_full(i), 3);
    
        nodes_matrix_cov = [points(index1_cov, :);
                points(index2_cov, :);
                points(index3_cov, :)];
    
        area_not_cov = area_not_cov + getAreaTriangle(nodes_matrix_cov/1000);
    end
