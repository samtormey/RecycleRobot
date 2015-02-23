% Given a moving belt, find the best feasible time and path
function [bft, bfp] = goal2belt_picker(sgp_index, sol A, maxiter)
% sgp_index: starting goal point index
% sol: starting octo location, 

belt_params = ConvBelt();
v = belt_params.velocity;
disc = belt_params.disc;
len2 = belt_params.len2;

% heuristic for gap spaces in a row along the belt
gap_size = 2*pi*len2/disc;

% loop through forward solutions in a row until we find 
% one reachable in time
count = 0;
% best feasible time
bft = Inf;
% best feasible path 
bfp = Inf;


while bfp == Inf && count <= maxiter
    count = count + 1;
    [the1p, the2p, the1n, the2n] = inverseThe(sol + [count*gap_size; 0]);
    [index1p, index2p] = getBestStoredIndices(the1p, the2p, disc);
    [index1n, index2n] = getBestStoredIndices(the1n, the2n, disc);
    maybe_best_time_p = A{index1p, index2p, sgp_index, 1};
    maybe_best_time_n = A{index1n, index2n, sgp_index, 1};
   
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
            bfp = A{index1p, index2p, sgp_index, 2};
        elseif n_or_p_better == 'n'
            bfp = A{index1n, index2n, sgp_index, 2};
        else
            disp('Uh-oh')
        end
    end
end
            
            
    
    
    


