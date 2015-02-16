function testing_init_guess

X0 = 1;
options.init = 1;
n = 20;
start = [pi/2; pi/2; zeros(4,1)];
finish = [0; pi; zeros(4,1)];


[X0 statePath stateVelocity d_delta T] = ...
    RealOptimalPathFind(start,finish,options,X0,n);

keyboard

end