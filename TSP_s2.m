% TSP Scnenario 2: minimise altitude changes

% Traveling Salesman Problem formulation 
% Path generation

altitudeFun = @(ZI, ZJ) altFun(ZI, ZJ);

% Generate the number of possible path based on the number of waypoints 
idxs_2 = nchoosek(1:nb_waypoints,2);

% Calculate the euclidian distance between viewpoints
dist_eucli = pdist(waypoints, 'euclidean');

% Calculate the custom distance
if strcmp(obj_s2, 'alt')
    dist_2 = pdist(waypoints, altitudeFun);
end

if strcmp(obj_s2, 'alt&path')
    dist_2 = pdist(waypoints, altitudeFun) + opti_ratio * dist_eucli ;
end

squaredist_2 = squareform(dist_2);
lendist_2 = length(dist_2);

G_2 = graph(idxs_2(:,1),idxs_2(:,2));
% figure
% hGraph_2 = plot(G_2,'XData',waypoints(:, 1),'YData',waypoints(:, 2), 'ZData', waypoints(:, 3), 'LineStyle','none','NodeLabel',{}, 'Marker','none', 'LineWidth',3, 'EdgeColor','b');
% hold on
% axis equal;
% legend('Surface', 'Cluster medoids', 'Viewpoints', 'Waypoints', 'Start point', 'Path');

% hold off

% Create the equality constraints matrix 
Aeq_2 = spalloc(nb_waypoints,length(idxs_2),nb_waypoints*(nb_waypoints-1)); % Allocate a sparse matrix
for ii = 1:nb_waypoints
    whichIdxs_2 = (idxs_2 == ii); % Find the trips that include stop ii
    whichIdxs_2 = sparse(sum(whichIdxs_2,2)); % Include trips where ii is at either end
    Aeq_2(ii,:) = whichIdxs_2'; % Include in the constraint matrix
end
beq_2 = 2*ones(nb_waypoints,1);  
disp('Equality constraint matrix done, starting solver');


% Binary bound
intcon_2 = 1:lendist_2; % The values in intcon indicate the components of the decision variable x that are integer-valued. intcon has values from 1 through numel(f).
lb_2 = zeros(lendist_2,1); % lower bound of the decision variable
ub_2 = ones(lendist_2,1); % upper bound of the decision variable

opts_2 = optimoptions('intlinprog','Display','off');
[x_tsp_2,costopt_2,exitflag_2,output_2] = intlinprog(dist_2,intcon_2,[],[],Aeq_2,beq_2,lb_2,ub_2,opts_2);

disp('Initial solver calculation done');

x_tsp_2 = logical(round(x_tsp_2));
% Start node = idxs(x_tsp, 1) and then end nodes
Gsol_2 = graph(idxs_2(x_tsp_2,1),idxs_2(x_tsp_2,2),[],numnodes(G_2));
% Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2)); % Also works in most cases

% hold on;
% highlight(hGraph_2,Gsol_2,'LineStyle','-');
% title('Solution without Subtours');

% Eliminate subtours 
tourIdxs_2 = conncomp(Gsol_2); % finds the connected components in the current graph
numtours_2 = max(tourIdxs_2); % number of subtours
fprintf('# of subtours: %d\n',numtours_2);

A_2 = spalloc(0,lendist_2,0); % Allocate a sparse linear inequality constraint matrix
b_2 = [];
while numtours_2 > 1 % Repeat until there is just one subtour
    % Add the subtour constraints
    b_2 = [b_2;zeros(numtours_2,1)]; % allocate b
    A_2 = [A_2;spalloc(numtours_2,lendist_2,nb_waypoints)]; % A guess at how many nonzeros to allocate
    for ii = 1:numtours_2
        rowIdx_2 = size(A_2,1) + 1; % Counter for indexing
        subTourIdx_2 = find(tourIdxs_2 == ii); % Extract the current subtour
%         The next lines find all of the variables associated with the
%         particular subtour, then add an inequality constraint to prohibit
%         that subtour and all subtours that use those stops.
        variations_2 = nchoosek(1:length(subTourIdx_2),2);
        for jj = 1:length(variations_2)
            whichVar_2 = (sum(idxs_2==subTourIdx_2(variations_2(jj,1)),2)) & ...
                       (sum(idxs_2==subTourIdx_2(variations_2(jj,2)),2));
            A_2(rowIdx_2,whichVar_2) = 1;
        end
        b_2(rowIdx_2) = length(subTourIdx_2) - 1; % One less trip than subtour stops
    end

    % Try to optimize again
    [x_tsp_2,costopt_2,exitflag_2,output_2] = intlinprog(dist_2,intcon_2,A_2,b_2,Aeq_2,beq_2,lb_2,ub_2,opts_2);
    x_tsp_2 = logical(round(x_tsp_2));
    Gsol_2 = graph(idxs_2(x_tsp_2,1),idxs_2(x_tsp_2,2),[],numnodes(G_2));
    % Gsol = graph(idxs(x_tsp,1),idxs(x_tsp,2)); % Also works in most cases
    
    % Visualize result
    % hGraph_2.LineStyle = 'none'; % Remove the previous highlighted path
    % highlight(hGraph_2,Gsol_2,'LineStyle','-', 'EdgeColor','yellow', 'LineWidth',1)
    % drawnow
    
    % How many subtours this time?
    tourIdxs_2 = conncomp(Gsol_2);
    numtours_2 = max(tourIdxs_2); % number of subtours
    fprintf('# of subtours: %d\n',numtours_2)
end

% disp(['Absolute gap: ', num2str(output_2.absolutegap)]);
% 
% % Calculate path length
% path_lenght_2 = sum(dist(x_tsp_2 == 1))/1e3;
% disp(['Path length (battery): ', num2str(path_lenght_2), ' m']);
% 
% % Calculate the altitude change
% alt_changes_2 = 0;
% for i = 1:length(waypointsOrdered_2)-1
%     alt_changes_2 = alt_changes_2 + abs(waypointsOrdered_2(i+1, 3) - waypointsOrdered_1(i, 3));
% end
% disp(['Overall altitude changes (battery): ', num2str(alt_changes_2), ' m']);
