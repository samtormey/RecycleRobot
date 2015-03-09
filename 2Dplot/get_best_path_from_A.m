[the1p, the2p, the1n, the2n] = inverseThe([octo.x; octo.y], len1, len2);
belt_params = ConvBelt();
v = belt_params.velocity;
disc = belt_params.disc;
dt = 2*pi/disc;
theta_vec = -pi+dt:dt:pi;


[the1p, the2p, the1n, the2n] = inverseThe([octo.x; octo.y], len1, len2);
belt_params = ConvBelt();
v = belt_params.velocity;
disc = belt_params.disc;
dt = 2*pi/disc;
theta_vec = -pi+dt:dt:pi;
% [~,ind1p] = min(abs(theta_vec - the1p))
% [~,ind2p] = min(abs(theta_vec - the2p))
% [x,y] = fkSCARA(theta_vec(ind1p),theta_vec(ind2p),len1,len2)

[~,ind1n] = min(abs(theta_vec - the1n))
[~,ind2n] = min(abs(theta_vec - the2n))
[xn,yn] = fkSCARA(theta_vec(ind1n),theta_vec(ind2n),len1,len2)


%% Works up to here, so selecting closest inverse angles is not issue
%% 


% controlp = A{ind1p,ind2p,sgp_index,2,2};
% timep = A{ind1p,ind2p,sgp_index,2,1};
controln = A{ind1n,ind2n,sgp_index,2,2};
timen = A{ind1n,ind2n,sgp_index,2,1};
% pathp = control_to_position(controlp, n, start, timep);
pathn = control_to_position(controln, n, start, timen);
% [xp,yp] = fkSCARA(pathp(n,1),pathp(n,2),len1,len2)
[xn,yn] = fkSCARA(pathn(n,1),pathn(n,2),len1,len2)




