function [index1, index2] = getBestStoredIndices(theta1, theta2, theta_vec)

if theta1 < -pi
    theta1 = theta1 + 2*pi;
end

if theta1 > pi
    theta1 = theta1 - 2*pi;
end

if theta2 < -pi
    theta2 = theta2 + 2*pi;
end

if theta2 > pi
    theta2 = theta2 - 2*pi;
end


% get indices on a discretized wheel going from -pi to pi. 

if theta1 < -pi
    theta1 = theta1 + 2*pi;
end
if theta1 > pi
    theta1 = theta1 - 2*pi;
end
if theta2 < -pi
    theta2 = theta2 + 2*pi;
end
if theta2 > pi
    theta2 = theta2 - 2*pi;
end

[~,index1] = min(abs(theta1 - theta_vec));
[~,index2] = min(abs(theta2 - theta_vec));

return











