% Traveling Salesman Problem formulation 
% Path generation
tic;
% Generate the number of possible path based on the number of waypoints 
idxs = nchoosek(1:nb_waypoints,2);

% Calculate all the distance
dist = pdist(waypoints, 'euclidean');
squaredist = squareform(dist);
lendist = length(dist);

G = graph(idxs(:,1),idxs(:,2));
% figure
% hGraph = plot(G,'XData',waypoints(:, 1),'YData',waypoints(:, 2), 'ZData', waypoints(:, 3), 'LineStyle','none','NodeLabel',{}, 'Marker','none', 'LineWidth',3, 'EdgeColor','b');
% hold on
% axis equal;
% legend('Surface', 'Cluster medoids', 'Viewpoints', 'Waypoints', 'Start point', 'Path');

% hold off

% Create the equality constraints matrix 
Aeq = spalloc(nb_waypoints,length(idxs),nb_waypoints*(nb_waypoints-1)); % Allocate a sparse matrix
for ii = 1:nb_waypoints
    whichIdxs = (idxs == ii); % Find the trips that include stop ii
    whichIdxs = sparse(sum(whichIdxs,2)); % Include trips where ii is at either end
    Aeq(ii,:) = whichIdxs'; % Include in the constraint matrix
end
beq = 2*ones(nb_waypoints,1);  
disp('Equality constraint matrix done, starting solver');

% Binary bound
intcon = 1:lendist; % The values in intcon indicate the components of the decision variable x that are integer-valued. intcon has values from 1 through numel(f).
lb = zeros(lendist,1); % lower bound of the decision variable
ub = ones(lendist,1); % upper bound of the decision variable

opts = optimoptions('intlinprog','Display','iter', 'RelativeGapTolerance',1e-2, 'CutGeneration', 'basic', 'IntegerPreprocess', 'none', 'Heuristics', 'advanced');

opts_2 = optimoptions('intlinprog', ...
    'Display', 'none', ... % No intermediate output
    'RelativeGapTolerance', 1e-1, ... % Accept solutions with up to 10% relative gap
    'AbsoluteGapTolerance', 1e-1, ... % Allow larger absolute gap
    'CutGeneration', 'none', ... % No cut generation
    'IntegerPreprocess', 'none', ... % Minimal integer preprocessing
    'Heuristics', 'none', ... % Avoid heuristics
    'MaxTime', 60, ... % Limit to 60 seconds
    'MaxNodes', 1e7, ... % Maximum number of nodes
    'ObjectiveCutOff', Inf, ... % No cutoff on objective
    'ConstraintTolerance', 1e-3, ... % Higher constraint tolerance
    'LPMaxIterations', 1e5, ... % Limit LP iterations
    'LPOptimalityTolerance', 1e-4, ... % Higher LP optimality tolerance
    'LPPreprocess', 'none', ... % No preprocessing for LP
    'MaxFeasiblePoints', Inf, ... % No limit on feasible points
    'NodeSelection', 'minobj', ... % Simpler node selection
    'ObjectiveImprovementThreshold', 1e-1, ... % Large threshold for objective improvement
    'OutputFcn', [], ... % No custom output functions
    'PlotFcn', [], ... % No custom plots
    'RootLPAlgorithm', 'dual-simplex', ... % Default LP algorithm
    'RootLPMaxIterations', 1e5 ... % Max iterations for root LP
);
% opts = optimoptions('ga','Display','iter');
[x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,[],[],Aeq,beq,lb,ub,opts_2);
% [x_tsp,costopt,exitflag,output] = ga(dist,intcon,[],[],Aeq,beq,lb,ub,opts);


initial_solver_s1 = toc;
cprintf('Red', 'Initial solver calculation done in %f seconds\n\n', initial_solver_s1);

x_tsp = logical(round(x_tsp));
% Start node = idxs(x_tsp, 1) and then end nodes
Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2),[],numnodes(G));
% Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2)); % Also works in most cases

% hold on;
% highlight(hGraph,Gsol,'LineStyle','-');
% title('Solution without Subtours');

% Eliminate subtours 
tourIdxs = conncomp(Gsol); % finds the connected components in the current graph
numtours = max(tourIdxs); % number of subtours
fprintf('# of subtours: %d\n',numtours);

A = spalloc(0,lendist,0); % Allocate a sparse linear inequality constraint matrix
b = [];
while numtours > 1 % Repeat until there is just one subtour
    % Add the subtour constraints
    b = [b;zeros(numtours,1)]; % allocate b
    A = [A;spalloc(numtours,lendist,nb_waypoints)]; % A guess at how many nonzeros to allocate
    for ii = 1:numtours
        rowIdx = size(A,1) + 1; % Counter for indexing
        subTourIdx = find(tourIdxs == ii); % Extract the current subtour
%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        variations = nchoosek(1:length(subTourIdx),2);
        for jj = 1:length(variations)
            whichVar = (sum(idxs==subTourIdx(variations(jj,1)),2)) & ...
                       (sum(idxs==subTourIdx(variations(jj,2)),2));
            A(rowIdx,whichVar) = 1;
        end
        b(rowIdx) = length(subTourIdx) - 1; % One less trip than subtour stops
    end

    % Try to optimize again
    opts_subtour = optimoptions('intlinprog','Display','iter', 'RelativeGapTolerance',5e-3, 'CutGeneration', 'basic', 'IntegerPreprocess', 'basic', 'Heuristics', 'advanced');
    [x_tsp,costopt,exitflag,output] = intlinprog(dist,intcon,A,b,Aeq,beq,lb,ub,opts_2);
    x_tsp = logical(round(x_tsp));
    Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2),[],numnodes(G));
    % Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2)); % Also works in most cases
    
    % Visualize result
    % hGraph.LineStyle = 'none'; % Remove the previous highlighted path
    % highlight(hGraph,Gsol,'LineStyle','-')
    % drawnow
    
    % How many subtours this time?
    tourIdxs = conncomp(Gsol);
    numtours = max(tourIdxs); % number of subtours
    fprintf('# of subtours: %d\n',numtours)
end