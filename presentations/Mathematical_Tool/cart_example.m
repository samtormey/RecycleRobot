function cart_example
%
% Show how to solve the minimum time problem for a cart moving on a line
% using the time variable transformation.
clf;

addpath /Users/nicklink/Documents/RecycleRobot/presentations/Mathematical_Tool

% initialize time discretization, X0 and the mass
n = 80;
dtau = 1/(n-1);
SZ = 3*n + 1;
X0 = zeros(3*n + 1,1); % X = [state (2n); control (n); time (1)]
mass = 10;

% forward euler = 0, backward = 1
back = 1;

% starting and ending states
start_state = [0 0]';
end_state = [100 0]';

% bounds on control
LB = -10; UB = 10;
lb = [-inf*ones(2*n,1); LB*ones(n,1); 0];
ub = [inf*ones(2*n,1); UB*ones(n,1); inf];

    % Function that returns the derivative of x_i, the state at one time
    % step
    function xprime = f(x_i,u);
        xprime = zeros(2,1);
        xprime(1) = x_i(2); 
        xprime(2) = u/mass;
    end

    % Constraints for fmincon
    function [cout,c_eq] = constraints(X)
        % function that returns the value of the constraints at a given
        % value of X. This function is called in fmincon and must equal 0
        % in order to satisfy the constraints.
        %
        % Inputs:  X  =   vector of decision variables
        %
        % Outputs: cout = empty, because there are no inequality
        % constraints
        %          c_eq = value of equality constraints for given value of
        %          X.
 
        c_eq = zeros(2*n+2,1); % vector of constraint values
        
        % dynamics constraints
        for i = 1:n-1
            % set indices of X to be called in constraint
            ind = 2*i-1:2*i;
            ind_p1 = 2*i+1:2*i+2;
            control = 2*n + i;
            control_p1 = 2*n + 1 + i;
            
            % forward euler
            if back == 0 
                % c = [x(i+1) - x(i)]/dt - T*f(x(i),u(i))
                c_eq(ind) = (X(ind_p1) - X(ind))/dtau ...
                    -X(end)*f(X(ind),X(control));
            
            % backward euler
            elseif back == 1 
                % c = [x(i+1) - x(i)]/dt - T*f(x(i+1),u(i+1))
                c_eq(ind) = (X(ind_p1) - X(ind))/dtau ...
                    - X(end)*f(X(ind_p1),X(control_p1));
            end
            
        end
             
        % start and end state constraints
        c_eq(end-3:end-2) = X(1:2) - start_state;
        c_eq(end-1:end) = X(2*n-1:2*n) - end_state;
        
        % need to set inequalities to empty. This must be returned for
        % fmincon to run.
        cout = [];
        
    end

    % Return the objective
    function b = Objective(X)
        
        b = X(end); % The last variable in x is the time
        
    end

    % make an initial guess that satisfies the constraints
    function b = init_guess(X)
        [~,b] = constraints(X);
    end

while(norm(init_guess(X0)) > .1)  
    fprintf('\n Finding random initial guess for X \n');
    X0 = rand(SZ,1);
    X0(1:2) = start_state;
    X0((2*(n-1) + 1):2*n) = end_state;
    X0 = fsolve(@init_guess,X0);
end

% set options for fmincon
opt.MaxFunEvals = 10000000;
tol = 1e-8; opt.TolFun = tol;

% solve our problem using fmincon!
[Xsol,fval,exitflag] = fmincon(@Objective,X0,[],[],[],[],lb,ub,@constraints,opt);

% extract out the position, control, and time from X
position = zeros(n,1);
control = zeros(n,1);
for i = 1:n
   position(i) = Xsol(2*i-1);
   control(i) = Xsol(2*n + i);
end

time = Xsol(end);

dt = Xsol(end)/(n-1);
fprintf('Solution found: Time = %f seconds \n',Xsol(end))


% plot
figure(1)
clf
plot(position(1),0,'o')
hold on
axis([-5,105,-0.1,0.1])
title('Movement of The Cart Over Time','Fontsize',12)
xlabel('Cart Position','Fontsize',12)

speed = 5;
pause(dt/speed)

for i = 2:n
   plot(position(i),0,'o')
   pause(dt/speed)
end

figure(2)

xx = [0:dt:Xsol(end)]';
plot(xx,control)

title('Control Over Time','Fontsize',12)
xlabel('Time','Fontsize',12)
ylabel('Control','Fontsize',12)

% if exitflag > 0
%     fprintf('\n Saving Path ... \n')
%     if back == 0
%         save(['./Cashed_Carts/FWDn=',num2str(n)],'n','mass',...
%         'position','control','dt','start_state','end_state','time','tol','back');
%     elseif back == 1
%         save(['./Cashed_Carts/BKWDn=',num2str(n)],'n','mass',...
%         'position','control','dt','start_state','end_state','time','tol','back');
%     end
% end
rmpath /Users/nicklink/Documents/RecycleRobot/presentations/Mathematical_Tool/

end




