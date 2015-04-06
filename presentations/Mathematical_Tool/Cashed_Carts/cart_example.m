function cart_example
%
% Show how to solve the minimum time problem for a cart moving on a line
% using the time variable transformation.
clc; clf;

n = 40;
dtau = 1/(n-1);
SZ = 3*n + 1;
X0 = zeros(3*n + 1,1);
mass = 10;

% starting and ending states
start_state = [0 0]';
end_state = [100 0]';

% bounds on control
LB = -10; UB = 10;
lb = [-inf*ones(2*n,1); LB*ones(n,1); 0];
ub = [inf*ones(2*n,1); UB*ones(n,1); inf];

    function b = f(x,u);
        
        % cart dynamics. b is the derivate of x
        b = zeros(2,1);
        b(1) = x(2);
        b(2) = u/mass;
        
    end

    function [cout,ceq] = constraints(x)
        
        ceq = zeros(2*n+2,1); % vector of constraint values
        
        % dynamics constraints
        for i = 1:n-1
            ind = 2*i-1:2*i;
            ind_fwd = 2*i+1:2*i+2;
            ind_control = 2*n + i;
            ceq(ind) = (x(ind_fwd) - x(ind))/dtau ...
                -x(end)*f(x(ind),x(ind_control));
        end
       
        
        % start and end state constraints
        ceq(end-3:end-2) = x(1:2) - start_state;
        ceq(end-1:end) = x(2*n-1:2*n) - end_state;
        
        % need to set inequalities to empty
        cout = [];
        
    end

    function b = Objective(x)
        
        b = x(end); % The last variable in x is the time
        
    end

    function b = init_guess(X)
        [~,b] = constraints(X);
    end

while(norm(init_guess(X0)) > .1)   % could save an X0 that works, save a few seconds
    fprintf('\n Finding random initial guess for X \n');
    X0 = rand(SZ,1);
    X0(1:2) = start_state;
    X0((2*(n-1) + 1):2*n) = end_state;
    X0 = fsolve(@init_guess,X0);
end

opt.MaxFunEvals = 1000000;
opt.TolFun = 1e-8;

[X,fval,exitflag] = fmincon(@Objective,X0,[],[],[],[],lb,ub,@constraints,opt);

position = zeros(n,1);
control = zeros(n,1);
for i = 1:n
   position(i) = X(2*i-1);
   control(i) = X(2*n + i);
end

dT = X(end)/(n-1);
dt = dT/5;
fprintf('Solution found in %f seconds \n',X(end))

% plot
figure(1)
plot(position(1),0,'o')
hold on
axis([-5,105,-0.1,0.1])
title('Movement of The Cart Over Time','Fontsize',12)
xlabel('Cart Position','Fontsize',12)

pause(dt)

for i = 2:n
   plot(position(i),0,'o')
   pause(dt)
end

figure(2)
xx = [0:dT:X(end)]';
plot(xx,control)


title('Control Over Time','Fontsize',12)
xlabel('Time','Fontsize',12)
ylabel('Control','Fontsize',12)

end




