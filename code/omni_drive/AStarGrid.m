function [pose_robot,route,numExpanded] = AStarGrid (input_map, start_coords, dest_coords)
% Run A* algorithm on a grid.
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
pose_robot=zeros(1,2);
cmap = [1 1 1; ...%1
    0 0 0; ... %2
    1 0 0; ...%3
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

% meshgrid will `replicate grid vectors' nrows and ncols to produce
% a full grid
% type `help meshgrid' in the Matlab command prompt for more information
parent = zeros(nrows,ncols);

% 
[X, Y] = meshgrid (1:ncols, 1:nrows);

xd = dest_coords(1);
yd = dest_coords(2);

% Evaluate Heuristic function, H, for each grid cell
% Manhattan distance
H = abs((X - xd) * (X - xd)) + abs((Y - yd)*(Y - yd));
H = H';
% Initialize cost arrays
f = Inf(nrows,ncols);
g = Inf(nrows,ncols);

g(start_node) = 0;
f(start_node) = H(start_node);

% keep track of the number of nodes that are expanded
numExpanded = 0;

% Main Loop

while true
    
    % Draw current map
    map(start_node) = 5;
    map(dest_node) = 6;
    
     %make drawMapEveryTime = true if you want to see how the
    % nodes are expanded on the grid. 
    drawMapEveryTime = true;
    if (drawMapEveryTime)
        %image(1.5, 1.5, map);
        image('XData',[0.5 99.5],'YData',[0.5 99.5],'CData',map);
        grid on;
        axis image;
        drawnow;
    end
    
    % Find the node with the minimum f value
    [min_f, current] = min(f(:));
    
    if ((current == dest_node) || isinf(min_f))
        break;
    end
    
    % Update input_map
    map(current) = 3;
    f(current) = Inf; % remove this node from further consideration
    
    % Compute row, column coordinates of current node
    [i, j] = ind2sub(size(f), current);
    
    % *********************************************************************
    % ALL YOUR CODE BETWEEN THESE LINES OF STARS
    % Visit all of the neighbors around the current node and update the
    % entries in the map, f, g and parent arrays
    %
     node=[i-1,j,(i>1 && i<=nrows);i+1,j,(i>=1 && i<nrows);...
        i,j-1,(j>1 && j<=ncols);i,j+1,(j>=1 && j<ncols);...
        i-1,j-1,(i>1 && j>1);i-1,j+1,(i>1 && j<ncols);...
        i+1,j-1,(i<nrows && j>1);i+1,j+1,(i<nrows && j<ncols)];
    if i>=1&&i<=nrows &&  j>=1&&j<=ncols
    for m=1:8
        if node(m,3)
        a=node(m,1);b=node(m,2);
            each_node= sub2ind(size(g),a,b);
              if map(each_node)~=2 && map(each_node)~=3 && map(each_node)~=5
               if g(each_node)>(g(current)+H(each_node)-H(current))
                  g(each_node)=g(current)+H(each_node)-H(current);
                  f(each_node)=g(each_node)+H(each_node);
                  parent(each_node)=current;
                  map(each_node)=4;
               end
              end 
        end   
    end
    end
     numExpanded=numExpanded+1;
    
    %*********************************************************************
    
    
end

%% Construct route from start to dest by following the parent links
if (isinf(f(dest_node)))
    route = [];
else
    route = [dest_node];
    [pose_robot(2),pose_robot(1)]=ind2sub(size(f),route(1));
    while (parent(route(1)) ~= 0)
        route = [parent(route(1)), route]; 
        [y, x] = ind2sub(size(f),route(1));
        %fprintf('%f %f\n',x,y);
        pose_robot=[x y; pose_robot];
    end
end
end
