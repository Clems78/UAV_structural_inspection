% Flight status identifyer
% Define condition to identify an input path as a particular flight status
% Thresholds for either vertical or horizontal paths 
% Past this threshol we consider it as a diagonal path up or down 
% 

% input = waypointsOrdered
% Output = flight status + associated parameters for each path section
% flight status possible : flying upwards or flying downwards
% 
function [flight_status, Dv, Dh, D] = flightStatusIdentifiyer(wp_1, wp_2)

    D = norm(wp_1 - wp_2);
    Dv = wp_2(3) - wp_1(3);
    Dh = sqrt(D^2 - Dv^2);

    if (Dv > 0)
        flight_status = 'diagonal_upwards';
    elseif (Dv == 0)
        flight_status = 'horizontal';
    else
        flight_status = 'diagonal_downwards';
        Dv = abs(Dv);
    end
end