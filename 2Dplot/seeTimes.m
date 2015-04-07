clear all

pit = load('Precompute/Controls_n=20_numThe=80_gps=5');
A = pit.A;
n = pit.n;
[num_goal_pts,~] = size(pit.goal_configs);

num_theta = 80;
dt = 2*pi/num_theta;
theta_vec = -pi+dt:dt:pi;
cnt = 1;

for th1 = 1:80
    for th2 = 1:80
        for gps = 1:8
            for dir = 1:2
                temp = A{th1, th2, gps, dir, 1};
                if isnan(temp) == 0
                    time(cnt) = temp;
                    cnt = cnt + 1;
                end
            end
        end
    end
end

keyboard

