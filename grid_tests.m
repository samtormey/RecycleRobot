options.xspace = 1:4:50;
options.yspace = 1:20;
options.theta1space = 0:3:20;
options.theta2space = -3:1:3;

[theta1,theta2,x,y] = ...
    get_closests_in_grid(8,2.1,2.5,3.6,options);