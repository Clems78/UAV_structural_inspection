% Pareto front for alt&length optimisation 
% algo = 'LKH'; % MILP or ACO or LKH
set(groot, 'defaultTextInterpreter', 'latex');  % Use LaTeX interpreter for all text objects
set(groot, 'defaultAxesTickLabelInterpreter', 'latex'); % Use LaTeX for tick labels
set(groot, 'defaultLegendInterpreter', 'latex'); % Use LaTeX for legends
set(groot, 'defaultAxesFontSize', 12);   % Adjust as necessary
set(groot, 'defaultTextFontSize', 12);   % Adjust as necessary
set(groot, 'defaultAxesFontWeight', 'normal');  % or 'bold' as needed


tic
algo = [3];

iter_value = 0.025;

% close all;

pareto_front_enabled = true;
i_pf = 1;
j_pf = 1;
opti_ratio = 0;
pareto_front = 1;
% path_length_2 = zeros(length(linspace(0, 1, 21)), 1);
% alt_changes_2 = zeros(length(linspace(0, 1, 21)), 1);
% E = zeros(length(linspace(0, 1, 21)), 1);*
path_length_pf = [];
alt_changes_pf = [];
E_pf = [];
TSP_duration_pf = [];

for i = 1:length(algo)

        algo_choice = algo(i);
    
    while (opti_ratio < 1 )
    
        opti_ratio = opti_ratio + iter_value
    
        switch algo_choice 
            case 1 
                main;
                path_length_pf(i_pf) = path_length_2;
                alt_changes_pf(i_pf) = round(alt_changes_2, 2)/1e3;
                E_pf(i_pf) = E;
                TSP_duration_pf(i_pf) = TSP_bat_consumption_duration;
                algo_name = "MILP";
            case 2
                Main_ACO_TSP;
                path_length_pf(i_pf) = path_lenght_ACO/1e3;
                alt_changes_pf(i_pf) = round(alt_changes_ACO, 2)/1e3;
                E_pf(i_pf) = E_ACO;
                TSP_duration_pf(i_pf) = ACO_duration;
                algo_name = "ACO";
            case 3
                lkh_run;
                path_length_pf(i_pf) = path_lenght_LKH/1e3;
                alt_changes_pf(i_pf) = alt_changes_LKH/1e3;
                E_pf(i_pf) = E_LKH;
                TSP_duration_pf(i_pf) = NaN;
                algo_name = "LKH";
        end
    
        opti_ratio_plot(i_pf) = opti_ratio;
        i_pf = i_pf + 1;
    
    end
    total_pf_time = toc;
    disp(['Total run time: ', num2str(round(total_pf_time, 1)), ' s']);

    
    pareto_front_enabled = false;

close all;

% Create the figure
figure(1);
hold on;

% Plot path length on the primary y-axis
yyaxis left;
plot(path_length_pf, alt_changes_pf, 'LineWidth', 3);
ylabel('Altitude changes (m)', 'FontSize', 12, 'Interpreter', 'latex');
% ylim([20, 40]);  % Adjust the y-axis limits to zoom in on the intersection area
ylim([0, 1100]);  % Adjust the y-axis limits to zoom in on the intersection area

% Plot E_pf on the secondary y-axis, converted to MJ
yyaxis right;
plot(path_length_pf, E_pf / 1000, 'LineWidth', 3);
ylabel('Energy Consumption (MJ)', 'FontSize', 12, 'Interpreter', 'latex');
% ylim([9, 19]);  % Adjust the y-axis limits to zoom in on the intersection area
ylim([0, 650]);  % Adjust the y-axis limits to zoom in on the intersection area

% Add a horizontal line at the minimum of E_pf (converted to MJ)
min_E_pf_MJ = min(E_pf) / 1000;  % Find the minimum value and convert to MJ
hline = yline(min_E_pf_MJ, '--r', 'LineWidth', 2);  % Add the horizontal line

% Adjust the x-axis limits to focus on the intersection area
xlim([0, 6000]);  % Adjust these limits based on the intersection region

% Add legend, title, and labels
legend({algo_name, 'Energy Consumption (MJ)', 'Minimum Energy Consumption'}, ...
    'FontSize', 10, 'Interpreter', 'latex');
title(['Pareto Front - ', algo_name], 'FontSize', 12, 'Interpreter', 'latex');
xlabel('Path length (m)', 'FontSize', 12, 'Interpreter', 'latex');

% Grid on for better visibility
grid on;



hold off;
    
    figure(2);
    hold on;
    plot(E_pf/1000);
    title('Energy Consumption');
    xlabel('optimisation ratio');
    ylabel('Energy consumtion (kJ)');
    set(gca, 'XTick', 1:numel(opti_ratio_plot), 'XTickLabel', opti_ratio_plot);
    legend(algo_name);
    hold off;
    
    
    figure(3);
    hold on;

    bar(TSP_duration_pf);
    title('Computing time');
    xlabel('optimisation ratio');
    ylabel('Computing time (s)');
    set(gca, 'XTick', 1:numel(opti_ratio_plot), 'XTickLabel', opti_ratio_plot);
    legend(algo_name);
    hold off;

end



