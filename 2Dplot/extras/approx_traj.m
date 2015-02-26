function X0 = approx_traj(n,T)  % accepts start and finish as row vectors



    start = [0 0 0 0 0 0]';
    finish = [-pi+1 -pi+1 0 0 0 0]';



    positions = zeros(3,n);
    velocities = zeros(3,n);
    accelerations = zeros(3,n);
    torques = zeros(6,n);
    a_2 = zeros(3,n);
    a_3 = zeros(3,n);
    torques = zeros(3,n);
    X0 = zeros(6*n+1,1);

    positions(:,1) = start(1:3);
    positions(:,n) = finish(1:3);
    velocities(:,1) = start(4:6);
    velocities(:,n) = finish(4:6);
    

    tn = T/(n-1);

    for i = 2:n-1
        positions(:,i) = positions(:,i-1) + (finish(1:3) - start(1:3))/n;  
    end
    for i = 2:n-1
        velocities(:,i) = (positions(:,i+1) - positions(:,i-1))/(2*tn);
    end
    for i = 1:n-1
        a_2(:,i) =  3/(tn^2)*(positions(:,i+1) - positions(:,i)) - ...
                        2/tn*velocities(:,i) - 1/tn*velocities(:,i+1);
        a_3(:,i) = -2/(tn^3)*(positions(:,i+1) - positions(:,i)) + ...
                        1/tn^2*(velocities(:,i+1) - velocities(:,i));
    end
    
    for i = 1:n-1
        accelerations(:,i) = 2*a_2(:,i) + 6*a_3(:,i)*tn;
    end 
       
    
    for i = 2:n
            
        plot3D_SCARA(positions(1,i),positions(2,i),0);            
        pause(.1)
            
    end   
        
    
    
    
    accelerations(:,n) = zeros(3,1);  % could change this potentially
% 
%     I = computeMoments;
%     
%     for i = 1:n
%         torques(:,i) = f_acc([positions(:,i); velocities(:,i)],accelerations(:,i));    
%     end    

%     
%     
% keyboard
%     
%     function torque = f_acc(X,acc)
% 
%         th1 = X(1);  th2 = X(2); d3  = X(3); 
%         th1d = X(4); th2d = X(5); d3d  = X(6);
% 
%         H = [I(14)+2*I(12)*cos(th1)+2*I(15)*cos(th2), .5*(I(17)+I(18)*cos(th2)), 0;
%             .5*(I(17)+I(18)*cos(th2)), I(16)+.5*I(13)*cos(th2), 0;
%             0, 0, I(19)];
%         h = [-2*I(15)*sin(th2)*th1d*th2d - .5*I(18)*sin(th2)*th2d^2;
%             I(15)*sin(th2)*th1d^2 - .25*I(13)*sin(th2)*th2d^2;
%             0];     
% 
%         torque = -H*acc + h; 
% 
%     end
% 
    for i = 1:n  
        jvi = (4*(i-1) + 1):4*i;  % indices of the 6 state variables at time step i
        cti = (4*n + 2*(i-1) + 1):(4*n + 2*i);  %
        X0(jvi,1) = [positions(1:2,i); velocities(1:2,i)];                
        X0(cti,1) = accelerations(1:2,i)/10;
    end
    
     X0(end) = T;
        
end




    function I = computeMoments

% Initialize robot
robot = ScaraInit();

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


%th1subd = -pi/2;  % desired final joint angles       
th2subd = -pi/2;
Kv = 30;  % constants for impedence controllers
Kp = 130;

Iz1 = l1^3/12;
Iz2 = Iz1;

r1 = l1/2;
r2 = l2/2;

% Moments
I(1) = mr1/(3 * l1) * ((l1 - cm1)^3 + cm1^3) + m1*(l1 - cm1)^2;
I(2) = mr2/(3 * l2) * ((l2 - cm2)^3 + cm2^3) + m2*(l2 - cm2)^2;


Mt3 = M3;  % Mt3 is mass of 3rd link including gripper and load mass
Mr2 = 0;  % rotor mass of motor in joint 2
n1 = 0;  % reduction rate in joint 1
n2 = 0;
Ir1 = 0;  % moment of rotor inertia of motor 1
Ir2 = 0;
            
I(1) = I(1);
I(2) = I(2);
I(3) = I(2) + (M2+M3)*cm2^2;  %(M2+M3) is the mass of 2nd and 3rd link and gripper and load
I(4) = I(1) + I(3) + n1*Ir1 + (Mr2+M2+M3)*l1^2;  % Here, M2 from the .pdf is M2+M3, as above
I(5) = M2*l1*cm2;
I(6) = I(3) + n2^2*Ir2;
I(7) = M3*l3^3/12;
I(10) = Mt3*(l1^2+l2^2);
I(8) = M3*(l1^2+l2^2)+I(7)+I(10);
I(9) = M3*l2^2+I(7);
I(11) = 2*M3*l2^2+2*I(7)+I(10);
I(12) = M3*l1*l2;
I(13) = Mt3*l1*l2;
I(14) = I(4) + I(8);
I(15) = I(5) + I(13);
I(16) = I(6) + I(9) + .25*I(10);
I(17) = I(3) + I(11);
I(18) = I(5) + 2*(I(12)+I(13));
I(19) = 1;  % M3

    end




