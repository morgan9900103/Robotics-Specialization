function [rads1,rads2] = computeRrInverseKinematics(X,Y)

syms theta1 theta2 ;

% Parameters
L1 = 1;
L2 = 1;
x = 0;
y = 1.5;


% Inverse Kinematics
theta2_1 = acos((x^2 + y^2 - L1^2 - L2^2)/2*L1*L2);
theta1_1 = atan2(1.5, 0) - atan2(L2*sin(theta2_1), L1+L2*cos(theta2_1));

theta2_2 = -acos((x^2 + y^2 - L1^2 - L2^2)/2*L1*L2);
theta1_2 = atan2(1.5, 0) + atan2(L2*sin(theta2_2), L1+L2*cos(theta2_2));



if ((0 < theta1_1) && (theta1_1 < pi/2))
    rads1 = theta1_1;
    rads2 = theta2_1;
else
    rads1 = theta1_2;
    rads2 = theta2_2;
end

