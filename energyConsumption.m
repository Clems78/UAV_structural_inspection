function E = energyConsumption(flight_status, varargin)
% Estimate the energy being consumed by the drone for a given maneuvres
% flight status : take-off, vertical flight upwards, vertical flight downwards,
% horizontal flight, hovering, 
% E = total energy consumed (Joules)
% V = take-off speed (m/s)
% D = distance (m)
% H = altitude (m)
% t = time (s)

p = inputParser;

% Define default values and validation functions for each parameter
defaultV = 0;
defaultH = 0;
defaultTime = 0;
defaultD = 0;

% Add required and optional parameters to the parser
addRequired(p, 'flight_status');
addOptional(p, 'V', defaultV, @isnumeric);
addOptional(p, 'H', defaultH, @isnumeric);
addOptional(p, 'time', defaultTime, @isnumeric);
addOptional(p, 'D', defaultD, @isnumeric);

% Parse the input arguments
parse(p, flight_status, varargin{:});

% Retrieve the values from the parser
flight_status = p.Results.flight_status;
V = p.Results.V;
H = p.Results.H;
t = p.Results.time;
D = p.Results.D;

    switch flight_status
        case 'take-off' 
            E = -0.432*V^2 + 3.786*V - 1.224;
        case 'hovering'
            E = (4.917*H + 275.204)*t;
        case 'horizontal'
            E = 308.709*t - 0.852;
        case 'upwards'
            E = 315*D - 211.261;
        case 'downwards'
            E = 68.956*D - 65.183;    
    end
end