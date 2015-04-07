function [control,closest_goal_ind,time,new_start] = belt2goal_picker(A,current_config,num_goal_pts)

% Output: [time,control,goal_point]

belt_params = ConvBelt();
v = belt_params.velocity;
disc = belt_params.disc;
dt = 2*pi/disc;
theta_vec = -pi+dt:dt:pi;

[ind1,ind2] = getBestStoredIndices(current_config(1),current_config(2),theta_vec);
maybe_best_time = zeros(num_goal_pts,1);

for i = 1:num_goal_pts
    
    maybe_best_time(i) = A{ind1, ind2, i, 1, 1};    
    
end

[~,closest_goal_ind] = min(maybe_best_time);

control = A{ind1,ind2,closest_goal_ind,1,2};
time = A{ind1,ind2,closest_goal_ind,1,1};

new_start = [theta_vec(ind1) theta_vec(ind2) 0 0]';
