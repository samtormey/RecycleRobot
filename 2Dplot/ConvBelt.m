function belt = ConvBelt

robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;

belt.robo2bottom = 0.1;
belt.robo2top = len1 + len2 - .5;
belt.robo2goal = len1 + len2 - .3;

% belt.velocity = 0.6;
belt.velocity = 0.3;
belt.disc = 80;
belt.num_rec = 4;
belt.rec_width = .3;
belt.left_right = 3;
belt.height = 0.2;

end