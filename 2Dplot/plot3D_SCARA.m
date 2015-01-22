function plot3D_SCARA (the1,the2,d3)
% Plot the 3D scara given:
%
% the1: The angle of the first scara arm
% the2: The angle of the second scara arm
% d3: The distance of the third scara arm.
robot = ScaraInit;

n = 10;
base_radius = 0.1; 
h = 2;
t = 0.2;
len1 = robot.l_1;
len2 = robot.l_2;
len3 = robot.l_3;

draw_base;
draw_arm_1;
draw_arm_2;
draw_arm_3;


    function draw_base
        r = base_radius;
        
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

    function draw_arm_1
        % h = height of base
        % t = thickness of arm
        % r = end radius
        % the1 = theta 1
        l = len1; r = base_radius;
        
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

    function draw_arm_2
        r = base_radius; 
        
        n = round(n/2)*2; % make n even
        num_verts = 2*n + 4;
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
        top_face = [n+3:num_verts, ones(1,n+2)*(2*n+4)];
        % side faces
        side_faces = zeros(n+2,2*n+4);
        for i = 1:n+1
            side_faces(i,1:4) = [i, i+1, i+n+3, i+n+2];
            side_faces(i,5:num_verts) = ones(1,2*n)*(i+n+2);
        end
        side_faces(end,1:4) = [1, n+2, 2*n+4, n+1];
        side_faces(end,5:num_verts) = ones(1,2*n)*(n+1);
        
        faces = [bottom_face; top_face; side_faces];
        
        
        % rotate vertices
        R1 = [cos(the1), -sin(the1), 0;
            sin(the1), cos(the1), 0;
            0        , 0        , 1];
        R2 = [cos(the2), -sin(the2), 0;
            sin(the2), cos(the2), 0;
            0        , 0        , 1];
        
        translate = [ones(num_verts,1)*(len1 - 2*r), zeros(num_verts,2)];
        translate = zeros(num_verts,3);
        vertices2 = (R1*(R2*vertices'))';
%         vertices2 = (R2*(vertices-translate)')';
        
        patch('Vertices',vertices2,'Faces',faces,'facecolor','b')
        
        
    end

    function draw_arm_3
        n = 10;
        r = base_radius/2;
        
        dtheta = 2*pi/n;
        theta = 0;
        
        % create vertices
        bottom_verts = zeros(n,3);
        top_verts = bottom_verts;
        for i = 1:n
            bottom_verts(i,:) = [r*cos(theta) + len1+len2 - 4*base_radius, r*sin(theta), h+2*t-d3];
            top_verts(i,:) = [r*cos(theta) + len1+len2 - 4*base_radius, r*sin(theta), h+2*t+len3-d3];
            
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
        
        patch('Vertices',vertices,'Faces',faces,'facecolor','b')
        
    end

end
