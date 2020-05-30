% Robotics: Estimation and Learning 
% WEEK 3
% 
% This script is to show how to use bresenham.
clear all
close all
map = zeros(30,30);
orig = [10,5]; % start point
% occ = [20 20];
occ = [15 1; 15 2; 15 3; 15 4; 15 5; 15 6; 15 7; 15 8; 15 9; 15 10; 14 10; 13 10; 12 10; 11 10; 10 10; 9 10; 8 10; 7 10; 6 10; 5 10; 5 9; 5 8; 5 7; 5 6; 5 5; 5 4; 5 3; 5 2; 5 1; 6 1; 7 1; 8 1; 9 1; 10 1; 11 1; 12 1; 13 1; 14 1];
% occ = [20 20; 20 19; 20 18; 20 17; 20 16; 20 15; 20 14; 20 13; 20 12; 20 11; 20 10; 19 20; 18 20; 17 20; 16 20; 15 20; 14 20];
% get cells in between
for i=1:size(occ,1)
    [freex, freey] = bresenham(orig(1),orig(2),occ(i,1),occ(i,2));  
    % convert to 1d index
    free = sub2ind(size(map),freey,freex);
    % set end point value 
    % map(occ(i,2),occ(i,1)) = 100;
    map(occ(i,2),occ(i,1)) = map(occ(i,2),occ(i,1)) + 1;
    % set free cell values
    % map(free) = -100;
    map(free) = map(free) - 0.5;
    if map(occ(i,2),occ(i,1)) > 3
        map(occ(i,2),occ(i,1)) = 3;
    end
    if map(free) < -3
        map(free) = -3;
    end
end



figure(1),
imagesc(map); hold on;
plot(orig(1),orig(2),'rx','LineWidth',3); % indicate start point
axis equal;
