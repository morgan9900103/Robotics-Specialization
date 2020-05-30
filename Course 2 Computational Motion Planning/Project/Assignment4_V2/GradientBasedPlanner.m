function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);

%%% All of your code should be between the two lines of stars.
% *******************************************************************
scale = sqrt(gx.^2 + gy.^2);
gx = gx ./ scale;
gy = gy ./ scale;

gx(scale == 0) = 0;
gy(scale == 0) = 0;

current_pos = start_coords;
route(1,:) = current_pos(:);

count = 1;

while ((norm(current_pos-end_coords) > 2) && (count < max_its))
    idx = sub2ind(size(f), round(current_pos(2)), round(current_pos(1)));
    current_pos = current_pos + [gx(idx), gy(idx)];
    count = count + 1;
    route(count, :) = current_pos(:);
end


% *******************************************************************
end
