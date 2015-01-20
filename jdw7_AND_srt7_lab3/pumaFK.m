function [T,puma_T] = pumaFK(joint_angles, puma)


L1 = puma.parameters.L_1;
L2 = puma.parameters.L_2;
L3 = puma.parameters.L_3;
L4 = puma.parameters.L_4;

a1 = puma.parameters.a1;
a2 = puma.parameters.a2;
a3 = puma.parameters.a3;
a4 = puma.parameters.a4;

T01 = dhtf(0,0,0,joint_angles(1));
T12 = dhtf(a1,0,0,joint_angles(2));
T23 = dhtf(0,L2,L3,joint_angles(3));
T34 = dhtf(a2,0,L4,joint_angles(4));
T45 = dhtf(a3,0,0,joint_angles(5));
T56 = dhtf(a4,0,0,joint_angles(6));

puma_T = {T01,T12,T23,T34,T45,T56};
T = T01*T12*T23*T34*T45*T56;

