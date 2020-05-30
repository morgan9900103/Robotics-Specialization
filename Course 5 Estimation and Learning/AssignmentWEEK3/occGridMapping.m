% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)

% ranges                - 1081 x 1000
% scanAngles            - 1081 x 1
% pose [x, y, theta]'    - 3    x 1000

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 

% the number of grids for 1 meter.
myResol = param.resol;
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2); % N = 1000
for j = 1:N % for each time,
    % Find grids hit by the rays (in the grid map coordinate)
    x = pose(1,j);
    y = pose(2,j);
    theta = pose(3,j);
    
    real_occ = [ranges(:,j).*cos(theta + scanAngles) -ranges(:,j).*sin(theta + scanAngles)] + [x y];
    grid_occ = ceil(myResol * real_occ) + myorigin';
    
    robot_grid_pos = ceil(myResol * [x y]) + myorigin';
    
    % Find occupied-measurement cells and free-measurement cells
    n = size(grid_occ,1);
    for i = 1:n % for each data
        [freex, freey] = bresenham(robot_grid_pos(1),robot_grid_pos(2),grid_occ(i,1),grid_occ(i,2));
        free = sub2ind(param.size,freey,freex);
        occ = sub2ind(param.size,grid_occ(i,2),grid_occ(i,1));

        myMap(int32(occ)) = min(lo_max, myMap(int32(occ))+lo_occ);
        myMap(free) = max(lo_min, myMap(free)-lo_free);
    end
end
end

