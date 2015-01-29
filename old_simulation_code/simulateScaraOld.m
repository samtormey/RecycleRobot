function [  ] = simulateScaraOld(  )


close all;

% Initialize robot
robot = ScaraInit();
pit = load('ScaraOwnMinPath.mat');
n = pit.n;
control = pit.control;
T = pit.T;

dt = T/n; % [s]]
t = 0:dt:T;

%M((numel(t)-1)/2) = struct('cdata',[],'colormap',[]);

X_dot(:,1) = [0 0 0]';
X(:,1) = [0 0 0]; 
accel = [0 0 0]';

i = 1;
for i = 1:1

% constants
m1 = robot.m_1;  % point mass of arm1 (the actuator for link 2 probably)
m2 = robot.m_2;
m3 = robot.m_3;
mr1 = robot.m_r1; % mass of link1, evenly distributed part
mr2 = robot.m_r2;
mr3 = robot.m_r3;

mr1 = 0;
mr2 = 0;  % first, for simplicity, assume that the joints are evenly distributed masses.
mr3 = 0;

l1 = robot.l_1; % length of link1
l2 = robot.l_2;
l3 = robot.l_3;

g = robot.g; % gravity

M1 = m1 + mr1;
M2 = m2 + mr2;
M3 = m3 + mr3;

cm1 = (mr1*l1/2 + m1*l1)/M1; % center of mass of link 1
cm2 = (mr2*l2/2 + m2*l2)/M2;
cm3 = (mr3*l3/2 + m3*l3)/M3;


% Initial Conditions


% Externally Applied Torque
tau_1 = 0 * ones(size(t)); % [N-m]
tau_2 = 0 * ones(size(t)); % [N-m]
for_3 = 0 * ones(size(t));

th1subd = -pi/2;  % desired final joint angles       
th2subd = -pi/2;
Kv = 30;  % constants for impedence controllers
Kp = 130;

Iz1 = l1^3/12;
Iz2 = Iz1;

r1 = l1/2;
r2 = l2/2;

% Moments
I1 = mr1/(3 * l1) * ((l1 - cm1)^3 + cm1^3) + m1*(l1 - cm1)^2;
I2 = mr2/(3 * l2) * ((l2 - cm2)^3 + cm2^3) + m2*(l2 - cm2)^2;
      
    Mt3 = M3;  % Mt3 is mass of 3rd link including gripper and load mass                   
    Mr2 = 0;  % rotor mass of motor in joint 2
    n1 = 0;  % reduction rate in joint 1
    n2 = 0;
    Ir1 = 0;  % moment of rotor inertia of motor 1
    Ir2 = 0;
            
   I1 = I1;
   I2 = I2;
   I3 = I2 + (M2+M3)*cm2^2;  %(M2+M3) is the mass of 2nd and 3rd link and gripper and load
   I4 = I1 + I3 + n1*Ir1 + (Mr2+M2+M3)*l1^2;  % Here, M2 from the .pdf is M2+M3, as above
   I5 = M2*l1*cm2;
   I6 = I3 + n2^2*Ir2;
            
            
   I3p = M3*l3^3/12;
   I10 = Mt3*(l1^2+l2^2);
   I8 = M3*(l1^2+l2^2)+I3p+I10;   
   I9 = M3*l2^2+I3p;
   
   I11 = 2*M3*l2^2+2*I3p+I10;
   I12 = M3*l1*l2;
   I13 = Mt3*l1*l2;
   I14 = I4 + I8;
   I15 = I5 + I13;
   I16 = I6 + I9 + .25*I10;
   I17 = I3 + I11;
   I18 = I5 + 2*(I12+I13);
   
    E = zeros(size(t));
    
end   % initalize all the variables.  

for i = 1:n-1 % time

   th1 = X(1,i);  % joint angles
   th2 = X(2,i);
   d3  = X(3,i);

   th1d = X_dot(1,i);  % joint velocities
   th2d = X_dot(2,i);
   d3d  = X_dot(3,i);
      
   H = [I14+2*I12*cos(th1)+2*I15*cos(th2), .5*(I17+I18*cos(th2)), 0;
       .5*(I17+I18*cos(th2)), I16+.5*I13*cos(th2), 0;
        0, 0, M3];
   h = [-2*I15*sin(th2)*th1d*th2d - .5*I18*sin(th2)*th2d^2;
       I15*sin(th2)*th1d^2 - .25*I13*sin(th2)*th2d^2;
       0];
   
     
   tau = control(i,:)';   % current control, row vector.

   accel_prev = accel; 
      
   accel = H\(tau - h);  % solving for acceleration

   
    if i > 0
      X_dot(:,i+1) = X_dot(:,i) + .5 * (accel_prev + accel) * dt;  % advancing velocity
      X(:,i+1) = X(:,i) + .5 * (X_dot(:,i) + X_dot(:,i+1)) * dt; % advancing position
    end
    
    % E(i) = .5 * (M1 * (cm1 * th1d)^2 + th1d^2 * I1 + ...  % collecting total energy
    %        M2 * ( cm2^2 * th2d^2 + th2d^2 * a) + ...
    %        I2 * ((cos(th2) * th1d)^2  + th2d^2)) + ...
    %        M2 * cm2 * sin(th2) * g;
    
    
end

%  X = pit.statePath';  %Uncomment if wanting to test the path that was generated
% from MinTimeControlOptimization

%keyboard

% Graphical Simulation
keyboard

robot.handles = drawScara(X(:,1),robot);
for i = 2:n
    angles = X(:,i);
    [~,robot_T] = ScaraFK(angles,robot);
    set(robot.handles(1),'Matrix',robot_T{1});
    set(robot.handles(2),'Matrix',robot_T{2});
    set(robot.handles(3),'Matrix',robot_T{3});
    drawnow;    
%     if mod(i,2) == 0
%         M(i/2) = getframe; 
%     end
    pause(.1); % adjustable pause in seconds
end

% % Plot Energy
% figure();
% plot(t,E);
% legend('Total Energy')

% Plot Output
figure();
plot(t, X(1,:), '-.');
hold on
plot(t, X(2,:));
legend('theta 1','theta 2');
ylim([-pi pi])


end
