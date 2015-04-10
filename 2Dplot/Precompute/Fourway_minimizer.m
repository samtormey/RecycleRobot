function Fourway_minimizer
% Take in the four different A matrices and find the minimum path across
% all of them.

pit = load('Precompute/Controls_n=20_numThe=80_gps=5');
A = pit.A;
n = pit.n;
robot = pit.robot;
num_theta = pit.num_theta;
goal_configs = pit.goal_configs;
belt = pit.belt;

pit = load('Precompute/Controls_n=20_numThe=80_gps=5_posshift1=0_posshift2=1.mat');
B = pit.A;

pit = load('Precompute/Controls_n=20_numThe=80_gps=5_posshift1=1_posshift2=1.mat');
C = pit.A;

pit = load('Precompute/Controls_n=20_numThe=80_gps=5_posshift1=1_posshift2=0.mat');
D = pit.A;



for k = 1:8
    for th1_i = 1:80
        for th2_i = 1:80
            for direction = 1:2
                time(1) = A{th1_i,th2_i,k,direction,1};
                time(2) = B{th1_i,th2_i,k,direction,1};
                time(3) = C{th1_i,th2_i,k,direction,1};
                time(4) = D{th1_i,th2_i,k,direction,1};
                
                [~,min_time_index] = min(time);
                                
                if min_time_index == 2
                    A{th1_i,th2_i,k,direction,1} = B{th1_i,th2_i,k,direction,1};
                    A{th1_i,th2_i,k,direction,2} = B{th1_i,th2_i,k,direction,2};
                    disp('got B')
                end
                if min_time_index == 3
                    A{th1_i,th2_i,k,direction,1} = C{th1_i,th2_i,k,direction,1};
                    A{th1_i,th2_i,k,direction,2} = C{th1_i,th2_i,k,direction,2};
                    disp('got C')                    
                end
                if min_time_index == 4
                    A{th1_i,th2_i,k,direction,1} = D{th1_i,th2_i,k,direction,1};
                    A{th1_i,th2_i,k,direction,2} = D{th1_i,th2_i,k,direction,2};
                    disp('got A')
                end
                       
            end
        end
    end
end

UnitedA = A;

save('./UnitedFriendMatrix','UnitedA',...
        'goal_configs','belt','n','robot','num_theta')
rmpath /Users/samtormey/matlab/RecycleRobot/2DPlot/
          