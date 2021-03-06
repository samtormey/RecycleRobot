% Given a moving belt, find the best feasible time and path
function [bfp, bft] = goal2belt_picker(sgp_index, sol, A, maxiter, alpha)
% sgp_index: starting goal point index, 2x1 vector
% sol: starting octo location, 
% A: pre-computed time and path storage
% maxiter = max iterations of guesses
%
% bft = best feasible time (goal to belt)
% bfp = best feasible path (goal to belt)

belt_params = ConvBelt();
robot = ScaraInit;
len1 = robot.l_1;
len2 = robot.l_2;


v = belt_params.velocity;
disc = belt_params.disc;
dt = 2*pi/disc;
theta_vec = -pi+dt:dt:pi;


% heuristic for gap spaces in a row along the belt

gap_size = 2*pi*len2/disc/alpha;

% loop through forward solutions in a row until we find 
% one reachable in time
count = 0;
% best feasible time
bft = Inf;
% best feasible path 
bfp = Inf;


while bfp == Inf && count <= maxiter
    count = count + 1;
    
    temp = sol + [count*gap_size; 0];
    
    if norm(temp) > len1 + len2
        % if the object is too far to the left, so it will
        % potentially be grabbable later
        if temp(1) < 0
            continue
            % if the object has fallen to far to the right, and is
            % now impossible to get
        else
            break
        end
    end
    
    [the1p, the2p, the1n, the2n] = inverseThe(sol + [count*gap_size; 0], ...
        len1, len2);
    
    [index1p, index2p] = getBestStoredIndices(the1p, the2p, theta_vec);
    [index1n, index2n] = getBestStoredIndices(the1n, the2n, theta_vec);
    
    maybe_best_time_p = A{index1p, index2p, sgp_index, 2, 1};
    maybe_best_time_n = A{index1n, index2n, sgp_index, 2, 1};
    
    if maybe_best_time_p < maybe_best_time_n
        maybe_best_time = maybe_best_time_p;
        n_or_p_better = 'p';
    else
        maybe_best_time = maybe_best_time_n;
        n_or_p_better = 'n';
    end
    
    if maybe_best_time < (gap_size*count)/v
        
        % best feasible time
        bft = maybe_best_time;
        % best feasible path
        if n_or_p_better == 'p'
            bfp = A{index1p, index2p, sgp_index, 2, 2};
        elseif n_or_p_better == 'n'
            bfp = A{index1n, index2n, sgp_index, 2, 2};
        else
            disp('Uh-oh')
        end
        
        %             norm(temp)
        %             keyboard
        break
    end
end
 

return
            
    
    
    


