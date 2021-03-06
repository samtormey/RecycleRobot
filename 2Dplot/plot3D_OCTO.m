function g = plot3D_OCTO(x,y,z,theta)

tic
n = 8;
% shift the angle space input by theta
t = linspace(0,2*pi,n)' + theta;
octocenter = [x y z];
oc = octocenter;

scale = 5;

verticesbottom = cat(2,cos(t)/scale+oc(1),sin(t)/scale+oc(2),ones(n,1)*(oc(3)-1/scale) + 0.2);
verticestop = cat(2,cos(t)/scale+oc(1),sin(t)/scale+oc(2),ones(n,1)*(oc(3)+1/scale));
vertices = cat(1,verticesbottom,verticestop);

top_bottom_faces = [1 2 3 4 5 6 7 8; 9 10 11 12 13 14 15 16];
% we have to repeat redundant coordinates to make faces a matrix
side_faces = ...
    [1 2 10 9 1 1 1 1;
     2 3 11 10 2 2 2 2;
     3 4 12 11 3 3 3 3;
     4 5 13 12 4 4 4 4;
     5 6 14 13 5 5 5 5;
     6 7 15 14 6 6 6 6;
     7 8 16 15 7 7 7 7;
     8 1 9 16 8 8 8 8];
faces = cat(1,top_bottom_faces,side_faces);
[num_faces, ~] = size(faces);

% pit = load('./draw/octos');
% [num_faces, ~] = size(faces);


g = patch('Vertices',vertices,'Faces',faces,...
        'FaceVertexCData',hsv(num_faces),'FaceColor','flat');

return