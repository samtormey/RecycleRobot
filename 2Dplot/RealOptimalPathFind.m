function [X statePath T exitflag comp_time] = RealOptimalPathFind(start,finish,options,X0,n)


% takes advantage of scopes in Matlab in order to reduce 
% number of inputs required for auxillary functions.
%
warning('off','all')

Q = 2;  % number of joints


startState = start;
finishState = finish;   % need to specify from which direction it moves.


n = 20;
SZ = 3*Q*n+1;   

dtau = 1/(n-1);   
dt = 1e-8;  % for finite difference in Jacobian
M = 10;
J = sparse((n+1)*2*Q,SZ);  % pre-allocation for own Jacobian

I = computeMoments;  % compute moments to be used in F and f, all constants

if n ~= 20
    disp('n is not 20')
    return    
end
    


%% Vestiges of old X0 paradigm

            % loads the previous solution X to test as new initial guess
            % 
            % temp = load('CurrentX0.mat');
            % X0_temp = temp.X;                   
            % if numel(X0_temp) == numel(X0)
            %     X0 = X0_temp;
            % end

            % % boundary conditions
            % X0(1:6) = start;  
            % X0((6*(n-1) + 1):6*n) = finish;
            
%%

back = 0;   %  Backwards or Forward Euler, own Jacobian only implemented for Forwards currently
yessave = 1;

        
    function b = f(X,u)

        th1 = X(1);  th2 = X(2); 
        th1d = X(3); th2d = X(4);

        H = [I(14)+2*I(12)*cos(th1)+2*I(15)*cos(th2), .5*(I(17)+I(18)*cos(th2));
            .5*(I(17)+I(18)*cos(th2)), I(16)+.5*I(13)*cos(th2)];
        h = [-2*I(15)*sin(th2)*th1d*th2d - .5*I(18)*sin(th2)*th2d^2;
            I(15)*sin(th2)*th1d^2 - .25*I(13)*sin(th2)*th2d^2];     
        b = [th1d; th2d; H\(h - u)];

    end  % robot dynamics

    % discretizied optimization function without computing Jacobian,
    % F_ownJacobian is more fully commented

    function b = F_dyn(X)

            b = zeros((n+1)*2*Q,1);               

            for i = 1:n-1            
                jvi = (2*Q*(i-1) + 1):2*Q*i;  % indices of the 2*Q state variables at time step i
                cti = (2*Q*n + Q*(i-1) + 1):(2*Q*n + Q*i); % indices of the Q control variables at time step i
                jvip1 = (Q*2*(i) + 1):2*Q*(i+1); % 2*Q state variables at time step i+1                   
                ctip1 = (2*Q*n + Q*(i) + 1):(2*Q*n + Q*(i+1));  % control variables at time step i+1  (used for backwards Euler)                  
                if back == 0        
                    feval = f(X(jvi),X(cti));  
                    b(jvi) = (X(jvip1) - X(jvi))/dtau - ... 
                        X(end)*feval;                                                  
                end            

                if back == 1
                    b(jvi) = (X(jvip1) - X(jvi))/dtau - ... 
                        X(end)*f(X(jvip1),X(ctip1));
                end 
            
        end

        jvn = (2*Q*(n-1) + 1):2*Q*n;
        b(end-(4*Q-1):end-(2*Q)) = startState - X(1:2*Q);
        b(end-2*Q+1:end) = finishState - X(jvn);     
        Cout = [];
        
    end   

    function [Cout,Ceq, Coutgrad, Ceqgrad] = F_ownJacobian(X)  % this has our hand structured Jacobian, not currently working.    
                 
            b = zeros((n+1)*2*Q,1);               

            for i = 1:n-1            
                jvi = (2*Q*(i-1) + 1):2*Q*i;  % indices of the 2*Q state variables at time step i
                cti = (2*Q*n + Q*(i-1) + 1):(2*Q*n + Q*i); % indices of the Q control variables at time step i
                jvip1 = (Q*2*(i) + 1):2*Q*(i+1); % 2*Q state variables at time step i+1                   
                ctip1 = (2*Q*n + Q*(i) + 1):(2*Q*n + Q*(i+1));  % control variables at time step i+1  (used for backwards Euler)                  
            
            % Forward Euler 
            if back == 0        
                evald_f = f(X(jvi),X(cti));   % save the value of f to reduce redundancy
                b(jvi) = (X(jvip1) - X(jvi))/dtau - ...  % x' = f  constraint, this is just F
                    X(end)*evald_f;    
                
            % Compute the Jacobian        
                
                % differentiate F(jvi) wrt to jvi
                for j = jvi
                    
                    % this is for computing finite differences
                    dtej = zeros(2*Q,1);  % directional infinitesimal
                    bjvi_dtej = zeros(2*Q,1);                    
                    k = mod(j,2*Q); if k == 0, k = 2*Q; end  % iterating through the 6 state variables, could be replaced with a counter 1:6
                    dtej(k) = dt; % creating directional infinitesimal
                    Xjvi_dtej = X(jvi) + dtej; % adding directional infinitesimal to THETA(i) 
                              
                    % derivative of b(i) wrt theta(i)
                    bjvi_dtej = - X(end)*(f(Xjvi_dtej,X(cti)) - evald_f)/dt;
                    bjvi_dtej(k) = bjvi_dtej(k) - 1/dtau;                                                
                    J(jvi,j) = bjvi_dtej;
                    
                    % derivative of b(i) wrt theta(i+1)
                    J(j,jvip1(k)) = 1/dtau;
                end
                for j = cti
                    
                    % prepare finite difference for the control variables
                    dtej = zeros(Q,1);
                    k = mod(j,Q); if k == 0, k = Q; end
                    dtej(k) = dt;                              
                    Ucti_dtej = X(cti) + dtej;
                    bjvi_dtej = -X(end)*(f(X(jvi),Ucti_dtej) - evald_f)/dt;  
                    J(jvi,j) = bjvi_dtej; 
                end
                for j = SZ
                    J(jvi,j) = -evald_f;
                end 
                                
            end            

            % Backwards Euler
            if back == 1
                b(jvi) = (X(jvip1) - X(jvi))/dtau - ... 
                    X(end)*f(X(jvip1),X(ctip1));
            end 
            
        end
        
        % Boundary conditions
        endstate = 2*Q*(n+1); 
        J(endstate-(4*Q-1):endstate,:) = 0;
        J(endstate-(4*Q-1):endstate-2*Q,1:2*Q) = -1*eye(2*Q,2*Q);
        J(endstate-2*Q+1:endstate,endstate-(4*Q-1):endstate-2*Q) = -1*eye(2*Q,2*Q);
        jvn = (2*Q*(n-1) + 1):2*Q*n;
        b(end-(4*Q-1):end-(2*Q)) = startState - X(1:2*Q);
        b(end-2*Q+1:end) = finishState - X(jvn);        
                
        % Wrappers for fmincon
        Ceq = b;
        Ceqgrad = J';
        Cout = [];       
        Coutgrad = [];
        
    end



    function X0 = fsolve_init(n)
    SZ = 9*n + 1;
    X0 = zeros(SZ,1);
    while(norm(F_dyn(X0)) > .1)   % could save an X0 that works, save a few seconds       
        fprintf('\n Finding random initial guess for X \n');
        X0 = rand(SZ,1);
        X0(1:2*Q) = startState;
        X0((2*Q*(n-1) + 1):2*Q*n) = finishState;
        X0 = fsolve(@F_dyn,X0);
    end
    end


    if options.init == 1
        [ X0,~,~,~ ] = simulateScara_controllers( startState, finishState, n, 4);
    end

    if options.init == 2
        fsolve_init(n);
    end 
    
%     if options.init == 3
%         X0 = approx_traj(n,2)
%     end
    
    % Lower and upperbounds on state variables
    JointLB = -inf*ones(2*Q*n,1); %      JointLB(5:3:end) = -3;  
    JointUB =  inf*ones(2*Q*n,1);  
    lb = [JointLB; -M*ones(Q*n,1); 0];
    ub = [JointUB; M*ones(Q*n,1); inf];
    opt = optimset('Algorithm','sqp','GradConstr','on','Display','off');
    opt.MaxFunEvals = 100000;
    opt.TolFun = 0.3; %*ones(SZ,n+2)';   % maybe increasing the tolerance would help?, size of b = F(X)
    
    tic
    [X,fval,exitflag] = fmincon(@Objective,X0,[],[],[],[],lb,ub,@F_ownJacobian,opt);
    comp_time = toc;
    fprintf('\n time = %1.3f',comp_time)
    if options.init == 1
       fprintf('  (Used controller initial guess)') 
    elseif options.init == 2
       fprintf('  (Used fsolve initial guess)') 
    end
    
    if exitflag ~= 1 && options.init ~= 1
       [X0,~,~,~ ] = simulateScara_controllers(startState, finishState, n, 4); 
       [X,fval,exitflag] = fmincon(@Objective,X0,[],[],[],[],lb,ub,@F_ownJacobian,opt);
    end
    if exitflag ~= 1 && options.init ~= 2
       X0 = fsolve_init(n);
       [X,fval,exitflag] = fmincon(@Objective,X0,[],[],[],[],lb,ub,@F_ownJacobian,opt);
    end
        
    
    d_delta = X(end) / (n-1);
%     fprintf('Time it takes to find optimal path: %f\n',time)
  
    
    for i = 1:Q
      statePath(:,i) = X(i:2*Q:2*Q*n);
      stateVelocity(:,i) = X((i+Q):2*Q:2*Q*n);
      control(:,i) = X(2*Q*n+i:Q:end-1);
      T = X(end);
    end  % extract state variables
    
    
%     for i = 1:n
%       plot3D_SCARA(statePath(i,1),statePath(i,2),0);
%       pause(.1);
%     end
%     control

end

% Extract the total time variable (the objective)

function b = Objective(x)

    b = x(end);     
    
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
I(19) = M3;  % M3

end








   