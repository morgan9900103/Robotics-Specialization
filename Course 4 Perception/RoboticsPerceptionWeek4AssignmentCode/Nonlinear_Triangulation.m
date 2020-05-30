function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
%% Nonlinear_Triangulation
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs:     

N = size(x1,1);
X = zeros(N,3);
for i = 1:N
    X(i,:) = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(i,:), x2(i,:), x3(i,:), X0(i,:));
end

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
    X = X0;
    
    for j = 1:25
    b = [x1(1) x1(2) x2(1) x2(2) x3(1) x3(2)]';
    
    Jf1 = Jacobian_Triangulation(C1, R1, K, X);
    Jf2 = Jacobian_Triangulation(C2, R2, K, X);
    Jf3 = Jacobian_Triangulation(C3, R3, K, X);
    J = [Jf1' Jf2' Jf3']';
    
    f1 = K*R1*(X'-C1);
    f2 = K*R2*(X'-C2);
    f3 = K*R3*(X'-C3);
    fx = [f1(1)/f1(3) f1(2)/f1(3) f2(1)/f2(3) f2(2)/f2(3) f3(1)/f3(3) f3(2)/f3(3)]';
    
    e = b - fx;
    deltaX = (J'*J)\J'*e;
    X = X + deltaX';
    end
end

function J = Jacobian_Triangulation(C, R, K, X)
    f = K*R*(X'-C);
    u = f(1);
    v = f(2);
    w = f(3);
    
    dXdx = K*R;
    dudx = dXdx(1,:);
    dvdx = dXdx(2,:);
    dwdx = dXdx(3,:);
    
    J = [(w*dudx-u*dwdx)/w^2; (w*dvdx-v*dwdx)/w^2];
end
