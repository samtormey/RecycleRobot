function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2014
% Lab 4
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

close all;

% Initialize robot
robot = RRInit();

% Time
dt = .01; % [s]
t_f = 10; % [s]

% Numerical Integration
t = 0:dt:t_f;
n = length(t);
X = zeros(2,n); % initialize variable to hold state vector
X_dot = zeros(2,n); % initialize variable to hold state vector derivatives

% constants
m1 = robot.m_1;
m2 = robot.m_2;
mr1 = robot.m_r1;
mr2 = robot.m_r2;
l1 = robot.l_1;
l2 = robot.l_2;
g = robot.g;

M1 = m1 + mr1;
M2 = m2 + mr2;

cm1 = (mr1*l1/2 + m1*l1)/M1;
cm2 = (mr2*l2/2 + m2*l2)/M2;



% Initial Conditions
X_dot(:,1) = [0 0]';
%X(:,2) = [0,0];
                        X(:,1) = [pi/3 pi/2];
accel = [0 0]';

% Externally Applied Torque
tau_1 = 0 * ones(size(t)); % [N-m]
tau_2 = 0* ones(size(t)); % [N-m]

                         th1subd = 0;
                         th2subd = pi/2;
                         Kv = 30;
                         Kp = 130;



% Moments
I1 = mr1/(3 * l1) * ((l1 - cm1)^3 + cm1^3) + m1*(l1 - cm1)^2;
I2 = mr2/(3 * l2) * ((l2 - cm2)^3 + cm2^3) + m2*(l2 - cm2)^2;

E = zeros(size(t));

for i = 1:length(t)-1

   th1 = X(1,i);
   th2 = X(2,i);


   th1d = X_dot(1,i);
   th2d = X_dot(2,i);
    
    a = (cm2 * cos(th2))^2 + 2 * cm2 * cos(th2) * l1 + l1^2;
    ad = -2*cm2^2*cos(th2)*sin(th2) - 2 * cm2 * sin(th2);
    M = [M1*cm1^2 + I1 + M2 * a + I2*cos(th2), 0; 0, M2*cm2^2 + I2];
    C = [th1d * (M2 * th2d * ad - I2 * sin(th2) * th2d);
        -.5 * (M2 * th1d^2 * ad - 2 * I2 * cos(th2) * sin(th2) * th1d^2)];
    G = [0; cos(th2) * M2 * cm2 * g];

    
    tau(1,1) = -Kp*(th1 - th1subd) - Kv*th1d;
    tau(2,1) = -Kp*(th2 - th2subd) - Kv*th2d;

    accel_prev = accel; 
    accel = M\(tau - C - G);
    
    % Trapezoidal Integration
    if i > 0
      X_dot(:,i+1) = X_dot(:,i) + .5 * (accel_prev + accel) * dt;
      X(:,i+1) = X(:,i) + .5 * (X_dot(:,i) + X_dot(:,i+1)) * dt;
    end
    
    E(i) = .5 * (M1 * (cm1 * th1d)^2 + th1d^2 * I1 + ...
        M2 * ( cm2^2 * th2d^2 + th2d^2 * a) + ...
        I2 * ((cos(th2) * th1d)^2  + th2d^2)) + ...
        M2 * cm2 * sin(th2) * g;
    
    
end

% Graphical Simulation
robot.handles = drawRR(X(:,1),robot);
for i = 2:length(t)
    setRR(X(:,i),robot);
    pause(1e-7); % adjustable pause in seconds
end

% Plot Energy
figure();
plot(t,E);
legend('Total Energy')

% Plot Output
figure();
plot(t, X(1,:), '-.');
hold on
plot(t, X(2,:));
legend('theta 1','theta 2');
ylim([-pi pi])


end
