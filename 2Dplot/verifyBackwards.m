close all
start = [0 0 0 0]';
finish = [1 2 0 0]';
n = 20;
X0 = zeros(6*n+1,1);
options.init = 1;


[X statePath T exitflag control] = RealOptimalPathFind(start,finish,options,X0,n);
 [ positions ] = simulateScara( controls, n, start, T);

for i = 1:n
%     plot3D_SCARA(statePath(i,1),statePath(i,2),0);
    plot3D_SCARA(positions(i,1),positions(i,2),0);
    pause(.1)
end


keyboard