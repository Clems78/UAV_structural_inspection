% Assess overlap
inspected_overlap = zeros(size(Mtar_filtered, 1), 1);

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
end

no_overlap = sum(inspected_overlap == 1) / size(Mtar_filtered, 1) * 100;
overlapped_twice = sum(inspected_overlap == 2) / size(Mtar_filtered, 1) * 100;
overlapped_thrice = sum(inspected_overlap == 3) / size(Mtar_filtered, 1) * 100;
overlapped_elmts = sum(inspected_overlap > 1) / size(Mtar_filtered, 1) * 100;


disp([num2str(no_overlap), '% of the polygons are inspected exactly once']);
disp([num2str(overlapped_elmts), '% of the polygons are inspected more than once']);
disp([num2str(overlapped_twice), '% of the polygons are inspected twice']);
disp([num2str(overlapped_thrice), '% of the polygons are inspected thrice']);


% Category labels for the bar plot
categories = {'No Overlap', 'Overlapped Elements', 'Overlapped Twice', 'Overlapped Thrice'};
percentages = [no_overlap, overlapped_elmts, overlapped_twice, overlapped_thrice];

% Create the bar plot
figure
bar(percentages);
% Set the x-tick labels using the category labels
set(gca, 'XTick', 1:numel(categories), 'XTickLabel', categories);

% Add a title and labels to the plot
title('Overlap Analysis');
xlabel('Overlap Status');
ylabel('Percentage (%)');

% Set the y-axis limits to [0, 100] to show percentages clearly
ylim([0 100]);

% Optionally, customize the bar colors
% bar_colors = [0 0.8 0; 1 0.6 0; 1 0 0; 0.8 0.8 0];  % Example color scheme
% set(gca, 'ColorOrder', bar_colors);