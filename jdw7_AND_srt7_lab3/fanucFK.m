function [T,fanuc_T] = fanucFK(joint_angles, fanuc)


L1 = fanuc.parameters.L_1;
L2 = fanuc.parameters.L_2;
L3 = fanuc.parameters.L_3;
L4 = fanuc.parameters.L_4;

a1 = fanuc.parameters.a1;
a2 = fanuc.parameters.a2;
a3 = fanuc.parameters.a3;
a4 = fanuc.parameters.a4;

T01 = dhtf(0,0,0,joint_angles(1));
T12 = dhtf(a1,L1,0,joint_angles(2) + pi/2);
T23 = dhtf(0,L2,0,joint_angles(3));
T34 = dhtf(a2,L3,L4,joint_angles(4));
T45 = dhtf(a3,0,0,joint_angles(5));
T56 = dhtf(a4,0,0,joint_angles(6));

fanuc_T = {T01,T12,T23,T34,T45,T56};
T = T01*T12*T23*T34*T45*T56;

