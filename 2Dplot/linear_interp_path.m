
Postry = zeros(3,n+2);
Veltry = Postry;
Acctry = Veltry;
Trqtry = Acctry;

Postry(:,1) = startState(1:3);
Postry(:,2) = startState(1:3);
Veltry(:,1) = zeros(3,1);
Veltry(:,end) = zeros(3,1);
Postry(:,end-1) = finishState(1:3);
Postry(:,end) = finishState(1:3);

% could try finding decent path with a controller.  

for i = 1:n
    Postry(:,i+2) = startState(1:3) + (finishState(1:3) - startState(1:3)).*(i/n);
    Veltry(:,i+1) = (Postry(:,i+2) - Postry(:,i+1))./dtau;
    Acctry(:,i) = (Veltry(:,i+1) - Veltry(:,i))./dtau;
    Xtemp = [Postry(:,i+2);Veltry(:,i+1)];    
    Trqtry(:,i) = f_acc(Xtemp,Acctry(:,i));  % kind of weird, Trq is non zero, but acceleration is zero ... I guess cause of h.
    jvi = (6*(i-1) + 1):6*i;  % indices of the 6 state variables at time step i
    cti = (6*n + 3*(i-1) + 1):(6*n + 3*i); 
    X0(jvi) = [Postry(:,i+2);Veltry(:,i+1)];
    X0(cti) = Trqtry(:,i);
end



    function acc = f_acc(X,acc)

        
        th1 = X(1);  th2 = X(2); d3  = X(3); 
        th1d = X(4); th2d = X(5); d3d  = X(6);

        H = [I(14)+2*I(12)*cos(th1)+2*I(15)*cos(th2), .5*(I(17)+I(18)*cos(th2)), 0;
            .5*(I(17)+I(18)*cos(th2)), I(16)+.5*I(13)*cos(th2), 0;
            0, 0, I(19)];
        h = [-2*I(15)*sin(th2)*th1d*th2d - .5*I(18)*sin(th2)*th2d^2;
            I(15)*sin(th2)*th1d^2 - .25*I(13)*sin(th2)*th2d^2;
            0];   
        
        acc = H*acc + h 
        
    end
        