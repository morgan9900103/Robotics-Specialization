% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the origin of the map in pixels
myorigin = param.origin;

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 1000;                           % Please decide a reasonable number of M, 
%                                based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
% P = repmat(myPose(:, 1), [1, M]);

thres = 0.8;

weights = ones(1, M) * (1/M);

noise_sigma = diag([0.01 0.01 0.035]);

for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
    M_prev = M;
    % 1) Propagate the particles 
    P = mvnrnd(myPose(:, j-1), noise_sigma, M)';
    
    % 2) Measurement Update 
    scores = zeros(1, M);
    
    d = ranges(:, j)';
    %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
    for m = 1:M % for each particle
        pose = P(:, m);
        grid_hits = [d.*cos(pose(3)+scanAngles)'; -d.*sin(pose(3)+scanAngles)'] + pose(1:2);
        occ = ceil(myResol * grid_hits) + myorigin;
        
        [freex, freey] = bresenham(pose(1),pose(2),occ(1),occ(2));
        free = [freex freey]';
        
        rm_occ = occ(1, :) > size(map, 2) | occ(2, :) > size(map, 1) | occ(1, :) < 1 | occ(2, :) < 1;
        rm_free = free(1, :) > size(map, 2) | free(2, :) > size(map, 1) | free(1, :) < 1 | free(2, :) < 1;
        
        occ(:, rm_occ) = [];
        occ = unique(sub2ind(size(map), occ(2, :), occ(1, :)));

        free(:, rm_free) = [];
        free = unique(sub2ind(size(map), free(2, :), free(1, :)));

        
        %   2-2) For each particle, calculate the correlation scores of the particles

        % LIDAR occupied, Map occupied
        scores(m) = scores(m) + 10 * sum(map(occ)  > 0.49);
        % LIDAR occupied, Map free
        scores(m) = scores(m) +  5 * sum(map(free) > 0.49);
        % LIDAR free, Map occupied
        scores(m) = scores(m) +  5 * sum(map(occ)  < 0.49);
        % LIDAR free, Map free
        scores(m) = scores(m) -  1 * sum(map(free) < 0.49);
    end

    %   2-3) Update the particle weights
    % update weights
    weights = weights .* scores;    
    % normalizing the weight so that sum is 1
    weights = weights/sum(weights); 
    
    %   2-4) Choose the best particle to update the pose
    [~, idx] = max(scores);
    myPose(:, j) = P(:, idx);

    % 3) Resample if the effective number of particles is smaller than a threshold


    % 4) Visualize the pose on the map as needed

end

end

