function [proj_points, t, R] = ar_cube(H,render_points,K)
%% ar_cube
% Estimate your position and orientation with respect to a set of 4 points on the ground
% Inputs:
%    H - the computed homography from the corners in the image
%    render_points - size (N x 3) matrix of world points to project
%    K - size (3 x 3) calibration matrix for the camera
% Outputs: 
%    proj_points - size (N x 2) matrix of the projected points in pixel
%      coordinates
%    t - size (3 x 1) vector of the translation of the transformation
%    R - size (3 x 3) matrix of the rotation of the transformation
% Written by Stephen Phillips for the Coursera Robotics:Perception course

% YOUR CODE HERE: Extract the pose from the homography
h1 = [H(1,1); H(2,1); H(3,1)];
h2 = [H(1,2); H(2,2); H(3,2)];
h3 = [H(1,3); H(2,3); H(3,3)];
r1 = h1/norm(h1);
r2 = h2/norm(h2);
r3 = cross(r1, r2);

[U, S, V] = svd([h1 h2 r3]);

R = U*[1 0 0; 0 1 0; 0 0 det(U*V')]*V';

t = h3/norm(h1);
%t(3) = abs(t(3));
%t = t/max([R(1,1) R(2,2) R(3,3)]);

% YOUR CODE HERE: Project the points using the pose
n = size(render_points, 1);
proj_points = zeros(n, 2);
for i = 1:n
    X_c = K*(R*(render_points(i,:)')+t);
    proj_points(i,1) = X_c(1)/X_c(3);
    proj_points(i,2) = X_c(2)/X_c(3);
end

end
