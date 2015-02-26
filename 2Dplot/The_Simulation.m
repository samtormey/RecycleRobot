function The_Simulation

close all
%%
belt = ConvBelt;
goal_y = belt.robo2goal;
belt_bottom = belt.robo2bottom;
belt_top = belt.robo2top;
blr = belt.left_right;
height = belt.height;
v = belt.velocity;
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;
num_rec = belt.num_rec;
rec_width = belt.rec_width;
h = zeros(num_rec,1);
%%
start_rec = linspace(-blr,blr,num_rec);

top_corners = [-blr belt_bottom 0;
               -blr belt_top 0;
                blr belt_top 0;
                blr belt_bottom 0];
            
bottom_corners =  [-blr belt_bottom -height;
               -blr belt_top -height;
                blr belt_top -height;
                blr belt_bottom -height];
            
verts = [top_corners; bottom_corners];

faces = [1 2 3 4 4 4 4 4; 5 6 7 8 8 8 8 8; 2 1 5 6 6 6 6 6; ...
    2 3 7 6 6 6 6 6; 3 4 8 7 7 7 7 7; 4 1 5 8 8 8 8 8];

plot3D_SCARA(0,0,0);
axis([-blr blr -blr blr 0 blr])
grid on
rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])

for i = 1:num_rec
    rec_vert(:,:,i) = [start_rec(i),belt_bottom,.1;
                        start_rec(i), belt_top,.1;
                        start_rec(i)+rec_width, belt_top,.1;
                        start_rec(i)+rec_width, belt_bottom,.1];
end

%octox = -blr;  
octox = 1.3;

start = [2.1256 0 0 0]';
sgp_index = 1;
ystart = 0.8;
sol = [octox; ystart];
pit = load('Precompute/Controls_n=20_numThe=20_gps=4.mat');
A = pit.A;
n = pit.n;
maxiter = 50;

[time,control] = goal2belt_picker(sgp_index, sol, A, maxiter);
dt = time/(n-1);
%  control = A{_,_,sgp_index,2,2};
%  time  = A{_,_,sgp_index,2,1};
 path = control_to_position(control, n, start, time);

disp(path)
keyboard

    for i = 1:100       
%         for j = 1:num_rec
%             if rec_vert(1,1,j) > blr
%                 rec_vert(:,1,j) = -blr*ones(4,1) + [0 0 rec_width rec_width]';
%             end
%             rec_vert(:,:,j) = rec_vert(:,:,j) + [.05*ones(4,1) zeros(4,2)]; 
%             if i > 1
%                  delete(h(j))
%             end
%             h(j) = patch(rec_vert(:,1,j),rec_vert(:,2,j),rec_vert(:,3,j));
%             pause(.001)
%         end
        octox = octox + v*dt;   
      
        if i < n+1
            plot3D_SCARA(path(i,1),path(i,2),-1)
            grid on
        end
        if i > 1
            delete(g);
        end
        g = plot3D_OCTO(octox,ystart,0,0);
        rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])
        pause(.1)
    end

patch('Vertices',verts,'Faces',faces,'facecolor',[.5 .5 .5]);
% 
% grid on
% rectangle('Position',[-blr,belt_bottom,2*blr,belt_top],'FaceColor',[.5 .5 .5])
% 
%     for i = 1:100
%         for j = 1:num_rec
%             if i > 1
% %                 delete(h(j))
%             end
%             h(j) = rectangle('Position',[start_rec(j)+(2*blr)/i,belt_bottom,rec_width,belt_top], ...
%                 'FaceColor',[.3,.3,.3]);
%             pause(.001)
%         end
%         pause(.1)
%     end
% 
% end

