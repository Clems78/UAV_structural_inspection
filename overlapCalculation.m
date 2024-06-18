% Assess overlap
inspected_overlap = zeros(size(Mtar_filtered, 1), 1);
area_overlaped = 0;
nodes_lists_filtered = nodes_list(ground_node == 0, :);
 


for i = 1:size(Mtar_filtered, 1)
    for j = 1:size(C, 1)
        distance_cluster = sqrt((centroid(i, 1) - C(j, 1))^2 + (centroid(i, 2) - C(j, 2))^2 + (centroid(i, 3) - C(j, 3))^2)/1000;
        within_range = distance_cluster < rmaj_p_2;
        
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

    if (inspected_overlap(i, 1) >= 2)
        % Getting the area covered by the polygon
        index1_overlap = nodes_lists_filtered(i, 1);
        index2_overlap = nodes_lists_filtered(i, 2);
        index3_overlap = nodes_lists_filtered(i, 3);
    
        nodes_matrix_overlap = [points(index1_overlap, :);
                points(index2_overlap, :);
                points(index3_overlap, :)];
    
        area_overlaped = area_overlaped + getAreaTriangle(nodes_matrix_overlap/1000);
    end


end

no_overlap = sum(inspected_overlap == 1) / size(Mtar_filtered, 1) * 100;
overlapped_twice = sum(inspected_overlap == 2) / size(Mtar_filtered, 1) * 100;
overlapped_thrice = sum(inspected_overlap == 3) / size(Mtar_filtered, 1) * 100;
overlapped_elmts = sum(inspected_overlap > 1) / size(Mtar_filtered, 1) * 100;