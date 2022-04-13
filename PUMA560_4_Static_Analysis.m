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
L(1) = Link([0 0 0 0],'modified');
L(2) = Link([0 0 0 -1.5708],'modified');
L(3) = Link([0 d3 a2 0],'modified');
L(4) = Link([0 d4 a3 -1.5708],'modified');
L(5) = Link([0 0 0 1.5708],'modified');
L(6) = Link([0 0 0 -1.5708],'modified');

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

%Final position of q
q1 = 1.0694; q2 = 0.0637; q3 = -0.9054; q4 = 0; q5 = 0.8417; q6 = -1.0694;
qf = [q1 q2 q3 q4 q5 q6];
qf = [1.0694 0.0637 -0.9054 0.0000 0.8417 -1.0694]; %INPUT

%Transformation Matrices & Rotation Matrices & Position Vectors
T01 = trotz(q1)*transl(0,0,0)*trotx(-pi/2);
R01 = tr2rt(T01);
O01 = transl(T01);
T12 = trotz(q2)*transl(a2,0,0)*trotx(0);
R12 = tr2rt(T12);
O12 = transl(T12);
T23 = trotz(q3)*transl(a3,0,d3)*trotx(-pi/2);
R23 = tr2rt(T23);
O23 = transl(T23);
T34 = trotz(q4)*transl(0,0,d4)*trotx(pi/2);
R34 = tr2rt(T34);
O34 = transl(T34);
T45 = trotz(q5)*transl(0,0,0)*trotx(-pi/2);
R45 = tr2rt(T45);
O45 = transl(T45);
T56 = trotz(q6)*transl(0,0,0)*trotx(0);
R56 = tr2rt(T56);
O56 = transl(T56);

R06 = R01*R12*R23*R34*R45*R56;

%Force and Moment Acting on the End Effector w.r.t base {0} coordinate
%system
f06 = [1;1;1];
n06 = [0.1;0.1;0.1];

%Force and Moment acting on the endeffector in its own cooordinate system
f66 = inv(R06)*f06;
n66 = inv(R06)*n06;

%Z Axis Vector
Z = [0;0;1];

%Force & Torque Vector Acting on each Joints
f55 = R56*f66;
f44 = R45*f55;
f33 = R34*f44;
f22 = R23*f33;
f11 = R12*f22;
f00 = R01*f11;

n55 = R56*n66+cross(O56,f55);
n44 = R45*n55+cross(O45,f44);
n33 = R34*n44+cross(O34,f33);
n22 = R23*n33+cross(O23,f22);
n11 = R12*n22+cross(O12,f11);
n00 = R01*n11+cross(O01,f00);

%Torque to be applied at the joints to maintain the equilibrium
tau5 = (n55.')*Z;
tau4 = (n44.')*Z;
tau3 = (n33.')*Z;
tau2 = (n22.')*Z;
tau1 = (n11.')*Z;
tau0 = (n00.')*Z;

%Force Jacobian Matrix
qf = [1.0694 0.0637 -0.9054 0.0000 0.8417 -1.0694]; %Configuration for which Jacobian Matrix to be calculated
Jf = (jacob0(PU,qf)).';

%Force-Torque Matrix
FT = [f06;n06];
tau = Jf*FT;

