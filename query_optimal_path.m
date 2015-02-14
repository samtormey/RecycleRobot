function [time, path] = query_optimal_path(theta1,theta2,x,y,options)

[x,y] = get_closests_in_grid(x,y,theta1,theta2,options);

% is the point within reach of the arm? 1 for true, 0 for false
is_within_arm_reach(x,y);




return


function [x,y] = get_closests_in_grid(x,y,theta1,theta2,options)

[~, mindex_x] = min(abs(x-options.xspace));
[~, mindex_y] = min(abs(y-options.yspace));
[~, mindex_theta1] = min(abs(theta1-options.theta1space));
[~, mindex_theta2] = min(abs(theta2-options.theta2space));


x = options.xspace(mindex_x);
y = options.yspace(mindex_y);
theta1 = options.theta1space(mindex_theta1);
theta2 = options.theta2space(mindex_theta2);

return
