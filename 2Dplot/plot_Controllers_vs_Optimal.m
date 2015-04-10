clear all
plot_eh = 1;
err = .1;
n = 20;
Kp = -40;
Kv = Kp/2;

M = 100;

startState = [.2 .5 0 0]';  % Example states
finishState = [2 2 0 0]';
 [ time ] = controllers_Approx ( startState, finishState, n, plot_eh, err, Kp, Kv, M);
 options.init = 1;
 X0 = 1;
 [X control T exitflag comp_time statePath] = RealOptimalPathFind(startState,finishState,options,X0,n, M);
  T
  time


% num_theta = 80;
% dtheta = 2*pi/num_theta;
% theta_vec = -pi+dtheta:dtheta:pi;
% 
% [index1, index2] = getBestStoredIndices(finishState(1), finishState(2), theta_vec);
% 
% [x,y] = fkSCARA(theta_vec(index1),theta_vec(index2), 1, 1);
% [x1,y1] = fkSCARA(theta_vec(index1),theta_vec(index2 + 1), 1, 1);
% norm([x,y] - [x1,y1])
% [x2,y2] = fkSCARA(theta_vec(index1 -1),theta_vec(index2), 1, 1);
% norm([x1,y1] - [x2,y2])
% norm([x2,y2] - [x,y])

