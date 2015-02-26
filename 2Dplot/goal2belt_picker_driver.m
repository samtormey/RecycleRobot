% goal2belt_picker_driver

belt = ConvBelt;
robot = ScaraInit;

len1 = robot.l_1;
len2 = robot.l_2;

sgp_index = 1;
sol = [0.5; 0.8];


pit = load('Precompute/Controls_n=20_numThe=20_gps=4.mat');
A = pit.A;

maxiter = 50;

[bft, bfp, b2g_bft, b2g_bfp] = goal2belt_picker(sgp_index, sol, A, maxiter);




