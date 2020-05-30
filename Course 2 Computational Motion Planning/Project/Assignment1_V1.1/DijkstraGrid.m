function [route,numExpanded] = DijkstraGrid (input_map, start_coords, dest_coords)
% Run Dijkstra's algorithm on a grid.
% Inputs : 
%   input_map : a logical array where the freespace cells are false or 0 and
%   the obstacles are true or 1
%   start_coords and dest_coords : Coordinates of the start and end cell
%   respectively, the first entry is the row and the second the column.
% Output :
%    route : An array containing the linear indices of the cells along the
%    shortest route from start to dest or an empty array if there is no
%    route. This is a single dimensional vector
%    numExpanded: Remember to also return the total number of nodes
%    expanded during your search. Do not count the goal node as an expanded node.


% set up color map for display
% 1 - white - clear cell
% 2 - black - obstacle
% 3 - red = visited
% 4 - blue  - on list
% 5 - green - start
% 6 - yellow - destination

cmap = [1 1 1; ...
        0 0 0; ...
        1 0 0; ...
        0 0 1; ...
        0 1 0; ...
        1 1 0; ...
	0.5 0.5 0.5];

colormap(cmap);

% variable to control if the map is being visualized on every
% iteration
drawMapEveryTime = true;

[nrows, ncols] = size(input_map);

% map - a table that keeps track of the state of each grid cell
map = zeros(nrows,ncols);

map(~input_map) = 1;   % Mark free cells
map(input_map)  = 2;   % Mark obstacle cells

% Generate linear indices of start and dest nodes
start_node = sub2ind(size(map), start_coords(1), start_coords(2));
dest_node  = sub2ind(size(map), dest_coords(1),  dest_coords(2));

map(start_node) = 5;
map(dest_node)  = 6;

% Initialize distance array
distanceFromStart = Inf(nrows,ncols);

% For each grid cell this array holds the index of its parent
parent = zeros(nrows,ncols);

distanceFromStart(start_node) = 0;

% keep track of number of nodes expanded 
numExpanded = 0;

% Main Loop
while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
    % make drawMapEveryTime = true if you want to see how the 
    % nodes are expanded on the grid. 
    if (drawMapEveryTime)
        image(1.5, 1.5, map);
        grid on;
        axis image;
        drawnow;
    end
    
    % Find the node with the minimum distance
    [min_dist, current] = min(distanceFromStart(:));
    
    if ((current == dest_node) || isinf(min_dist))
        break;
    end;
    
    % Update map
    map(current) = 3;         % mark current node as visited
    distanceFromStart(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(distanceFromStart), current);
    
   % ********************************************************************* 
    % YOUR CODE BETWEEN THESE LINES OF STARS
    
    % Visit each neighbor of the current node and update the map, distances
    % and parent tables appropriately.
    
%     A = [];
%     [r, c] = size(map);
%     index = sub2ind(size(map), i, j);
%     
%     % Neighbor on the left
%     if (index - r >= 1)
%         A = [A,index - r];
%     end
%     
%     % Neighbor on the top
%     if (index - 1 > r*(j - 1))
%         A = [A, index - 1];
%     end
%     
%     % Neighbor on the bottom
%     if (index + 1 <= r * j)
%         A = [A,index + 1];
%     end
%     
%     % Neighbor on the right
%     if (index + r <= (r * c))
%         A = [A,index + r];
%     end
% 
%     for v = 1:numel(A)
%         if (map(A(v)) == 1 || map(A(v)) == 4|| map(A(v)) == 6)
%             if(distanceFromStart(A(v)) >= min_dist + 1)
%                 distanceFromStart(A(v)) = min_dist + 1;
%                 parent(A(v)) = current;
%             end
% 
%             map(A(v)) = 4;
%         end
%     end
%     numExpanded = numExpanded + 1;

    A = [];
    index = sub2ind(size(map), i, j);
    
    if (index - 1 <= nrows*(j-1))
        A = [A, index - 1 + nrows];
    else
        A = [A, index - 1];
    end
    
    if (index + 1 > nrows*j)
        A = [A, index + 1 - nrows];
    else
        A = [A, index + 1];
    end
    
    if (index - nrows <= 0)
        A = [A, index - nrows + nrows*ncols];
    else
        A = [A, index - nrows];
    end
    
    if (index + nrows > nrows*ncols)
        A = [A, index + nrows - nrows*ncols];
    else
        A = [A, index + nrows];
    end
    
    for v = 1:numel(A)
        if (map(A(v)) == 1 || map(A(v)) == 4|| map(A(v)) == 6)
            if(distanceFromStart(A(v)) >= min_dist + 1)
                distanceFromStart(A(v)) = min_dist + 1;
                parent(A(v)) = current;
            end

            map(A(v)) = 4;
        end
    end
    
    numExpanded = numExpanded + 1;

    
    %*********************************************************************

end

%% Construct route from start to dest by following the parent links
if (isinf(distanceFromStart(dest_node)))
    route = [];
else
    route = [dest_node];
    
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route];
    end
    
        % Snippet of code used to visualize the map and the path
    for k = 2:length(route) - 1        
        map(route(k)) = 7;
        pause(0.1);
        image(1.5, 1.5, map);
        grid on;
        axis image;
    end
end

end