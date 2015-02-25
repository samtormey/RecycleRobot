function [index1, index2] = getBestStoredIndices(theta1, theta2, disc)

index1 = round(theta1*disc/(2*pi));
index2 = round(theta2*disc/(2*pi));

% dt = 2*pi/disc;
% index1 = round(theta1+pi-dt)/dt;
% index2 = round(theta2+pi-dt)/dt;

return











