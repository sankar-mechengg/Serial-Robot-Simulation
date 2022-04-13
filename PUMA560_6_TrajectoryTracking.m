clear all;
close all;

%Trajectory
q11 = deg2rad(160); q22 = deg2rad(25); q33 = deg2rad(-50); q44 = deg2rad(10); q55 = deg2rad(45); q66 = deg2rad(-60);
qf1 = [q11 q22 q33 q44 q55 q66];
q0 = [0 0 0 0 0 0];
tf = 10;
t = 0:0.15:tf; %Time 0 to 5 seconds in the increment of 0.15 INPUT
Q = jtraj(q0,qf1,t); %Create Joint Angles from initial to final joint configuration

mdl_puma560;
% view angle
ae = [138 8];

%PUMA 560
%Link Parameters INPUT
a2 = 0.4318 ; d3 = 0.15; a3 = 0.0203; d4 = 0.4318;
syms q1 q2 q3 q4 q5 q6

%DH Parameters
%Standard or Classic
L(1) = Link([0 0 0 -1.5708],'standard');
L(2) = Link([0 0 a2 0],'standard');
L(3) = Link([0 d3 a3 -1.5708],'standard');
L(4) = Link([0 d4 0 1.5708],'standard');
L(5) = Link([0 0 0 -1.5708],'standard');
L(6) = Link([0 0 0 0],'standard');

%Modified
% Order q(theta) d a alpha
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
Tr = PU.fkine(Q);

%To show the trajectory of the robot
for i = 1:1:length(t)
    T = Tr(i);
    trs = transl(T); %Takes the last column of Translation Matrix of T
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);
end

p560.plot3d(Q,'view',ae);
hold on;
plot3(xx,yy,zz,'color',[1 0 0],'LineWidth',2);
hold off;
