function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly
% 

N = size(X,1);
A = [];
for i = 1:N
    x_til = [x(i,:) 1]';
    x_til = (K\x_til)';
    skew = Vec2Skew(x_til);
    PX = [X(i,:) 1 zeros(1,4) zeros(1,4);...
          zeros(1,4) X(i,:) 1 zeros(1,4);...
          zeros(1,4) zeros(1,4) X(i,:) 1];
    temp = skew*PX;
    A = [A; temp];
end

[U, D, V] = svd(A);
P(1,:) = V(1:4,end)';
P(2,:) = V(5:8,end)';
P(3,:) = V(9:12,end)';

R(1,:) = P(1,1:3);
R(2,:) = P(2,1:3);
R(3,:) = P(3,1:3);

t = P(:,4);

[U, D, V] = svd(R);
R = U*V';
if det(U*V') > 0
    R = U*V';
    t = t/D(1,1);
elseif det(U*V') < 0
    R = -U*V';
    t = -t/D(1,1);
end

C = -R'*t;
