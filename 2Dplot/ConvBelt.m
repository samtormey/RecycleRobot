function belt = ConvBelt

addpath /Users/samtormey/matlab/RecycleRobot/2DPlot/
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;
rmpath /Users/samtormey/matlab/RecycleRobot/2DPlot/

belt.robo2bottom = 0.1;
belt.robo2top = len1 + len2 - .5;
belt.robo2goal = len1 + len2 - .3;

belt.velocity = 1;
belt.disc = 60;

end