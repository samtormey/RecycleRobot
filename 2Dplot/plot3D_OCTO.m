function plot3D_OCTO(x,y,z,theta)

n = 8;
% shift the angle space input by theta
t = linspace(0,2*pi,n)' + theta;
octocenter = [x y z];
oc = octocenter;
verticesbottom = cat(2,cos(t)+oc(1),sin(t)+oc(2),













return