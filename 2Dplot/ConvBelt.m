function belt = ConvBelt

addpath /Users/samtormey/matlab/RecycleRobot/2DPlot/
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;
rmpath /Users/samtormey/matlab/RecycleRobot/2DPlot/

belt.small_goal_y = len1 + len2 - .5;
belt.big_goal_y = len1 + len2 - .1;
belt.small_belt_y = .2;
belt.big_belt_y = len1 + len2 - .7;

end