clear all;
close all;


%PUMA 560
%Link Parameters INPUT
a2 = 0.4318 ; d3 = 0.15; a3 = 0.0203; d4 = 0.4318;
syms q1 q2 q3 q4 q5 q6

%DH Parameters
%Standard or Classic
% Order q(theta) d a alpha
L(1) = Link([0 0 0 -1.5708],'standard');
L(2) = Link([0 0 a2 0],'standard');
L(3) = Link([0 d3 a3 -1.5708],'standard');
L(4) = Link([0 d4 0 1.5708],'standard');
L(5) = Link([0 0 0 -1.5708],'standard');
L(6) = Link([0 0 0 0],'standard');

%Modified
% L(1) = Link([0 0 0 0],'modified');
% L(2) = Link([0 0 0 -1.5708],'modified');
% L(3) = Link([0 d3 a2 0],'modified');
% L(4) = Link([0 d4 a3 -1.5708],'modified');
% L(5) = Link([0 0 0 1.5708],'modified');
% L(6) = Link([0 0 0 -1.5708],'modified');

%Link Movement Limits in terms of Joint angle variables
L(1).qlim = [deg2rad(-160) deg2rad(160)];
L(2).qlim = [deg2rad(-225) deg2rad(45)];
L(3).qlim = [deg2rad(-45) deg2rad(225)];
L(4).qlim = [deg2rad(-110) deg2rad(110)];
L(5).qlim = [deg2rad(-100) deg2rad(100)];
L(6).qlim = [deg2rad(-266) deg2rad(266)];

%Build the Robot
PU = SerialLink(L);
PU.name = 'PUMA 560';

%Jacobian Matrix
qf = [1.0694 0.0637 -0.9054 0.0000 0.8417 -1.0694]; %Configuration for which Jacobian Matrix to be calculated
J = jacob0(PU,qf)
rank(J)
det(J)