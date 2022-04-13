clear all;
close all;


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

% Forward Kinematics
q = [q1 q2 q3 q4 q5 q6]; %INPUT
qf = [1.0694 0.0637 -0.9054 0.0000 0.8417 -1.0694]; %Final Position to Reach INPUT
qrest = [0 -pi/4 0 0 0 0]; %Rest Configuration INPUT
Tf = PU.fkine(qrest); %Transformation matrix from Forward Kinematics
PU.plot(qrest);

%Position in space
p = [0.9 0.9 0.9];

Tf.t(1,1) = 0.1;
Tf.t(2,1) = 0.2;
Tf.t(3,1) = 0.01;

%Inverse Kinematics
q0 = [0 0 0 0 0 0]; %Initial Joint Configuration INPUT
qinv = PU.ikine(Tf,q0,'mask',[1 1 1 1 1 1]); %Inverse Kinematics
PU.plot(qinv);


%Trajectory
t = 0:0.15:5; %Time 0 to 5 seconds in the increment of 0.15 INPUT
Q = jtraj(q0,qinv,t); %Create Joint Angles from initial to final joint configuration
Tr = PU.fkine(Q);

%To show the trajectory of the robot
for i = 1:1:length(t)
    T = Tr(i);
    trs = transl(T); %Takes the last column of Translation Matrix of T
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);
end

plot3(xx,yy,zz,'color',[1 0 0],'LineWidth',2);
hold on;
plot(PU,Q);
hold off;

