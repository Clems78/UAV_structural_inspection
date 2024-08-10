% Pareto front for alt&length optimisation 

i_pf = 1;
opti_ratio = 0;
pareto_front = 1;
path_length_2 = zeros(length(linspace(0, 1, 21)), 1);
alt_changes_2 = zeros(length(linspace(0, 1, 21)), 1);
E = zeros(length(linspace(0, 1, 21)), 1);

while (opti_ratio < 1.05)
    opti_ratio = opti_ratio + 0.025;
    main;
    i_pf = i_pf + 1;

end