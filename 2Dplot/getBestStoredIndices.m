function [index1, index2] = getBestStoredIndices(theta1, theta2, theta_vec)

% get indices on a discretized wheel going from -pi to pi. 

[~,index1] = min(abs(theta1 - theta_vec));
[~,index2] = min(abs(theta2 - theta_vec));

return











