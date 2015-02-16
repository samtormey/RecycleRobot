function plotDriver(statePath)

for i = 1:size(statePath,1)
    
    plot3D_SCARA(statePath(i,1),statePath(i,2),0);
    grid on
    pause(.1)
    
end