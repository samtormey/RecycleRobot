function plot3D_SCARA (the1,the2,d3)
% Plot the 3D scara given:
%
% the1: The angle of the first scara arm
% the2: The angle of the second scara arm
% d3: The distance of the third scara arm.
n = 10;
r = 0.5; 
h = 2;

draw_base(n,r,h)


end

function draw_base(n,r,h)
% n: number of points in the polygon
% r: radius of cylinder 
% h: height of base

dtheta = 2*pi/n;
theta = 0; 

% create vertices
bottom_verts = zeros(n,3);
top_verts = bottom_verts;
for i = 1:n  
    bottom_verts(i,:) = [r*cos(theta), r*sin(theta), 0];
    top_verts(i,:) = [r*cos(theta), r*sin(theta), h];
  
    theta = theta + dtheta;
end
     
vertices = [bottom_verts; top_verts];

top_face = 1:n;
bottom_face = n+1:2*n;

side_faces = zeros(n);
for i = 1:n-1
    side_faces(i,1:4) = [i, i+1, i+n+1, i+n];
    side_faces(i,5:n) = ones(1,n-4)*(i+n);
end
side_faces(n,1:4) = [2*n, n+1, 1, n];
side_faces(n,5:n) = ones(1,n-4)*n;

faces = [top_face; bottom_face; side_faces];
figure(1); clf;

patch('Vertices',vertices,'Faces',faces,'facecolor','b')
view([45 45])

end

