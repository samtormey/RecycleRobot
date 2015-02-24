function driverRealOptimal

% funList = {@Timer,@callinRealOptimal};
% matlabpool open 
% 
% parfor i=1:length(funList)
%     %# call the function
%     funList{i}(dataList{i});
% end
% 
% end

% function callinRealOptimal

    n = 20;
    Q = 2;
    X0 = zeros(9*n+1,1);
    options.init = 1;
    options.MaxFunEvals = 10000;

    startState = [.3 2.6 0 0]';  % Example states
    finishState = [0 0 0 0]';
    num = 50;
    dt = 1/num;
    
    control_fwd = zeros(n,2);
    control_bkwd = zeros(n,2);

    finishState = finishState + [2 1 0 0 ]';
    [X_fwd statePath T output] = RealOptimalPathFind(startState,finishState,options,X0,n);
    [X_bkwd statePath_bkwd T_bkwd output_bkwd] = RealOptimalPathFind(finishState,startState,options,X0,n);
    
    %statePath_bkwd_rev = -statePath_bkwd(,:);
    %         control_fwd
    for i = 1:Q
        control_fwd(:,i) = X_fwd(2*Q*n+i:Q:end-1);
        control_bkwd(:,i) = X_bkwd(2*Q*n+i:Q:end-1);
    end
    
    keyboard
    %         output
    %         options.init = 0;
    
    
% end

% function Timer




    
    