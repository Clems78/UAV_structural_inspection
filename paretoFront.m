% Pareto front for alt&length optimisation 
% algo = 'LKH'; % MILP or ACO or LKH

algo = [2, 3];

iter_value = 0.1;

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
    
    while (opti_ratio < 1 + iter_value)
    
        disp(["optimisation ratio = %f", opti_ratio]);
        opti_raqatio = opti_ratio + iter_value;
    
        switch algo_choice 
            case 1 
                main;
                path_lenght_pf(i_pf) = path_length_2;
                alt_changes_pf(i_pf) = round(alt_changes_2, 2)/1e3;
                E_pf(i_pf) = E;
                TSP_duration_pf(i_pf) = TSP_bat_consumption_duration;
                algo_name = "MILP";
            case 2
                Main_ACO_TSP;
                path_lenght_pf(i_pf) = path_lenght_ACO/1e3;
                alt_changes_pf(i_pf) = round(alt_changes_ACO, 2)/1e3;
                E_pf(i_pf) = E_ACO;
                TSP_duration_pf(i_pf) = ACO_duration;
                algo_name = "ACO";
            case 3
                lkh_run;
                path_lenght_pf(i_pf) = path_lenght_LKH/1e3;
                alt_changes_pf(i_pf) = round(alt_changes_LKH, 2)/1e3;
                E_pf(i_pf) = E_LKH;
                TSP_duration_pf(i_pf) = NaN;
                algo_name = "LKH";
        end
    
        opti_ratio_plot(i_pf) = opti_ratio;
        i_pf = i_pf + 1;
    
    end
    
    pareto_front_enabled = false;
    
    figure(1);
    hold on;
    plot(alt_changes_pf, path_lenght_pf);
    legend(algo_name);
    title('Pareto front', algo_name);
    xlabel('Altitude changes (m)');
    ylabel('Path length (m)');
    % zlabel('Z');
    grid on;
    hold off;
    
    figure(2);
    hold on;
    bar(E_pf);
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



