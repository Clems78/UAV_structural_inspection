% EC_path_tester
% defaultV = 0;
% defaultH = 0;
% defaultTime = 0;
% defaultDv = 0;
% defaultDh = 0;
% defaultL = 0;
% energyConsumption('upwards', V, H, t, Dv, Dh, L);

E = energyConsumption('armed', 0, 0, 5, 0, 0, 0);
E = E + energyConsumption('take-off', 1, 0, 0, 0, 0, 0);
E = E + energyConsumption('upwards', 0, 0, 0, 5, 0, 0);
E = E + energyConsumption('hovering', 0, 5, 10, 0, 0, 0);
E = E + energyConsumption('horizontal', 1, 0, 0, 0, 10, 0);
E = E + energyConsumption('downwards', 0, 0, 0, 5, 0, 0);
E = E + energyConsumption('payload', 0, 0, 0, 0, 0, 0);
E = round(E/1e3, 3);

disp(['Total energy consumed = ', num2str(E) ' kJ']);

% Result = 7.87 kJ ; Paper = same 
