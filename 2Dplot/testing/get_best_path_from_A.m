[the1p, the2p, the1n, the2n] = inverseThe([octo.x; octo.y], len1, len2);
belt_params = ConvBelt();
v = belt_params.velocity;
disc = belt_params.disc;
dt = 2*pi/disc;
theta_vec = -pi+dt:dt:pi;
[ind1p, ind2p] = getBestStoredIndices(the1p, the2p, theta_vec);
[ind1n, ind2n] = getBestStoredIndices(the1n, the2n, theta_vec);
controlp = A{ind1p,ind2p,sgp_index,2,2};
timep = A{ind1p,ind2p,sgp_index,2,1};
controln = A{ind1n,ind2n,sgp_index,2,2};
timen = A{ind1n,ind2n,sgp_index,2,1};
pathp = control_to_position(controlp, n, start, timep);
pathn = control_to_position(controln, n, start, timen);
[x,y] = fkSCARA(pathp(n,1),pathp(n,2),len1,len2)
[x,y] = fkSCARA(pathn(n,1),pathn(n,2),len1,len2)




