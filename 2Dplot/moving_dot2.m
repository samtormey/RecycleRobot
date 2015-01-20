pos = linspace(0,1,100);

figure
axis([-1 1 -1 1])

for i = 1:length(pos)
    plot(0,pos(i),'o','Markersize',50,'MarkerFaceColor','b')    
    axis([-1 1 -1 1])
    pause(.1)
end
