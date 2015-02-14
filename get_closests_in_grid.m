function [theta1,theta2,x,y] = ...
    get_closests_in_grid(theta1,theta2,x,y,options)

[~, mindex_x] = min(abs(x-options.xspace));
[~, mindex_y] = min(abs(y-options.yspace));
[~, mindex_theta1] = min(abs(theta1-options.theta1space));
[~, mindex_theta2] = min(abs(theta2-options.theta2space));


x = options.xspace(mindex_x);
y = options.yspace(mindex_y);
theta1 = options.theta1space(mindex_theta1);
theta2 = options.theta2space(mindex_theta2);

return