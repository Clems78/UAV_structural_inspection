function E = energyConsumptionPath(waypoints, V)
% Compute the energy consumed on a given paths
    E = 0;
    for i = 1:length(waypoints)-1
        wp_1 = waypoints(i, :);
        wp_2 = waypoints(i+1, :);
        [flight_status, Dv, Dh, D] = flightStatusIdentifiyer(wp_1, wp_2);
        E = E + energyConsumptionCalculation(flight_status, V, 0, 0, Dv, Dh, 0);
    end
    E = round(E/1e3, 1);
    cprintf('Text','Estimated energy consumption: %.1f kJ\n\n', E);

end