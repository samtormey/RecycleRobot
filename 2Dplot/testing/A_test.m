% Unit test
% Check that A performs correctly in 1 different cases

robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;

belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;

num_theta = 20;


dt = 2*pi/num_theta;
theta_vec = -pi+dt:dt:pi;

pit = load('Precompute/Controls_n=20_numThe=20_gps=4.mat');
A = pit.A;
n = pit.n;

gps = 4;

goal_width = 2*sqrt((len1+len2)^2 - goal_y^2);
goal_points_x = linspace(-goal_width/2,goal_width/2,gps);
goal_points_y = goal_y*ones(1,gps);
points = [goal_points_x; goal_points_y];
% Inverse Kinematics
[the1p, the2p, the1n, the2n] = inverseThe1_2(points,len1,len2);
goal_configs = [the1p the1n(2:end-1); the2p the2n(2:end-1)]'; % no
options.init = 1;
% X0 will be killed
X0 = 0;





% Tests begin

disp('Case 1 A:')
disp(A{3,3,1,1,2})

disp('Case 1 re-derived:')
[~, control, ~, ~, ~] = RealOptimalPathFind([theta_vec(3); theta_vec(3); 0; 0], ...
    [goal_configs(1,1); goal_configs(1,2); 0; 0], options, X0, n);
disp(control)
    
    

disp('Case 2 A:')
disp(A{17,4,1,2,2})


disp('Case 2 re-derived:')
[~, control, ~, ~, ~] = RealOptimalPathFind([goal_configs(1,1); goal_configs(1,2); 0; 0], ...
    [theta_vec(17); theta_vec(4); 0; 0], options, X0, n);
disp(control)
    

    



    

