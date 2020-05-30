function [elbow,endeff] = computeRrForwardKinematics(rads1,rads2)
%%GIVEN THE ANGLES OF THE MOTORS, return an array of arrays for the
%%position of the elbow [x1,y1], and endeffector [x2,y2]

% Parameters
L1 = 1;
L2 = 1;
theta1 = 1;
theta2 = 0.75

x1 = L1*cos(theta1);
y1 = L1*sin(theta1);
x2 = x1 + L2*cos(theta1 + theta2);
y2 = y1 + L2*sin(theta1 + theta2);

elbow = [x1,y1];
endeff =[x2,y2];
