function [the1p, the2p, the1n, the2n] = inverseThe(circlePositions, len1, len2)
    
    xx = circlePositions(1,:);
    yy = circlePositions(2,:);
    c2 = (xx.^2 + yy.^2 - len1^2 - len2^2) / ...
                (2*len1*len2);
     
    if abs(c2) > 1
        disp('circle Position is out of reachable workspace')
        return
    end
            
    %c2 = cos(inside2);
    s2p = sqrt(1-c2.^2);
    s2n = -sqrt(1-c2.^2);
     
    the2p = atan2(s2p,c2);
    the2n = atan2(s2n,c2);
    
    
    k1 = len1 + len2*c2;
    k2p = len2*s2p;
    k2n = len2*s2n;
    
    the1p = atan2(yy,xx) - ...
              atan2(k2p,k1);
         
          %keyboard
          
    the1n = atan2(yy,xx) - ...
              atan2(k2n,k1);    

end
