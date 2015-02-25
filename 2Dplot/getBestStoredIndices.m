function [index1, index2] = getBestStoredIndices(theta1, theta2, disc)

% get indices on a discretized wheel going from -pi to pi. 

index1 = round(theta1*disc/(2*pi));
index2 = round(theta2*disc/(2*pi));

return











