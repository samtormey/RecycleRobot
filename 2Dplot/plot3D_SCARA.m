function plot3D_SCARA (the1,the2,d3)
% Plot the 3D scara given:
%
% the1: The angle of the first scara arm
% the2: The angle of the second scara arm
% d3: The distance of the third scara arm.
global len1 len2 width
n = 10;
base_radius = width/2; 
base_height = 2;
t = 0.5;
length_1 = len1;
length_2 = len2;

draw_base(n,base_radius,base_height)
draw_arm_1(n,length_1,base_height,base_radius,t,the1)
draw_arm_2(n,length_1,length_2,base_height,base_radius,t,the1,the2)

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
axis([-2 2 -2 2 0 5])

end

function draw_arm_1(n,l,h,r,t,the1)
% n = number of vertices in polygons at ends
% l = length of arm
% h = height of base
% t = thickness of arm
% r = end radius
% the1 = theta 1

n = round(n/2)*2; % make n even

% theta is the angle used to make the polygon, not to be confuse with the1,
% the angle of the first arm
theta = pi/2;
dtheta = 2*pi/n;

%create vartices
bottom_verts = zeros(n+2,3);
top_verts = bottom_verts;

% first joint vertices
for i = 1:(n/2+1)
    bottom_verts(i,:) = [r*cos(theta), r*sin(theta), h];
    top_verts(i,:) = [r*cos(theta), r*sin(theta), h+t];
  
    theta = theta + dtheta;
end

theta = 3*pi/2;
% second joint vertices
for i = (n/2)+2:n+2
    bottom_verts(i,:) = [r*cos(theta) + l - 2*r, r*sin(theta), h];
    top_verts(i,:) = [r*cos(theta) + l - 2*r, r*sin(theta), h+t];
  
    theta = theta + dtheta;
end

vertices = [bottom_verts; top_verts];

% create faces
bottom_face = [1:n+2, ones(1,n+2)*(n+2)];
top_face = [n+3:2*n+4, ones(1,n+2)*(2*n+4)];
% side faces
side_faces = zeros(n+2,2*n+4);
for i = 1:n+1
   side_faces(i,1:4) = [i, i+1, i+n+3, i+n+2];
   side_faces(i,5:2*n+4) = ones(1,2*n)*(i+n+2);
end
side_faces(end,1:4) = [1, n+2, 2*n+4, n+1];
side_faces(end,5:2*n+4) = ones(1,2*n)*(n+1);

faces = [bottom_face; top_face; side_faces];


% rotate vertices
R = [cos(the1), -sin(the1), 0;
    sin(the1), cos(the1), 0;
    0        , 0        , 1];
vertices2 = (R*vertices')';

patch('Vertices',vertices2,'Faces',faces,'facecolor','b')


end

function draw_arm_2(n,len1,len2,h,r,t,the1,the2)
% n = number of vertices in polygons at ends
% len1 = length of arm 1
% len2 = length of arm 2
% h = height of base
% t = thickness of arm
% r = end radius
% the1 = theta 1
% the2 = theta 2

n = round(n/2)*2; % make n even

% theta is the angle used to make the polygon, not to be confuse with the1,
% the angle of the first arm
theta = pi/2;
dtheta = 2*pi/n;

%create vartices
bottom_verts = zeros(n+2,3);
top_verts = bottom_verts;

% first joint vertices
for i = 1:(n/2+1)
    bottom_verts(i,:) = [r*cos(theta) + len1 - 2*r, r*sin(theta), h+t];
    top_verts(i,:) = [r*cos(theta) + len1 - 2*r, r*sin(theta), h+2*t];
  
    theta = theta + dtheta;
end

theta = 3*pi/2;
% second joint vertices
for i = (n/2)+2:n+2
    bottom_verts(i,:) = [r*cos(theta) + len1 + len2 - 4*r, r*sin(theta), h+t];
    top_verts(i,:) = [r*cos(theta) + len1 + len2 - 4*r, r*sin(theta), h+2*t];
  
    theta = theta + dtheta;
end

vertices = [bottom_verts; top_verts];

% create faces
bottom_face = [1:n+2, ones(1,n+2)*(n+2)];
top_face = [n+3:2*n+4, ones(1,n+2)*(2*n+4)];
% side faces
side_faces = zeros(n+2,2*n+4);
for i = 1:n+1
   side_faces(i,1:4) = [i, i+1, i+n+3, i+n+2];
   side_faces(i,5:2*n+4) = ones(1,2*n)*(i+n+2);
end
side_faces(end,1:4) = [1, n+2, 2*n+4, n+1];
side_faces(end,5:2*n+4) = ones(1,2*n)*(n+1);

faces = [bottom_face; top_face; side_faces];


% rotate vertices
R1 = [cos(the1), -sin(the1), 0;
    sin(the1), cos(the1), 0;
    0        , 0        , 1];
R2 = [cos(the2), -sin(the2), 0;
    sin(the2), cos(the2), 0;
    0        , 0        , 1];

vertices2 = (R2*R1*vertices')';

patch('Vertices',vertices2,'Faces',faces,'facecolor','b')


end


