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
    X0 = zeros(9*n+1,1);
    options.init = 1;
    options.MaxFunEvals = 10000;

    startState = [.3 2.6 0 0 0 0]';  % Example states
    finishState = [0 0 0 0 0 0]';
    num = 50;
    dt = 1/num;

    for i = 1:num
        finishState = finishState + [dt*i dt*i 0 0 0 0]';
        [X0 statePath T output] = RealOptimalPathFind(startState,finishState,options,X0,n);
        statePath
%         output
%         options.init = 0;
    end
    
% end

% function Timer




    
    