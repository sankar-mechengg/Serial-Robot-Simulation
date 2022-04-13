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

%Initial Positio of q
q1 = 0; q2 = 0; q3 = 0; q4 = 0; q5 = 0; q6 = 0;
q0 = [q1 q2 q3 q4 q5 q6];
q0 = [0 0 0 0 0 0]; %INPUT
qi = [0 0 0 0 0 0];

%Final position of q
q1 = 1.0694; q2 = 0.0637; q3 = -0.9054; q4 = 0; q5 = 0.8417; q6 = -1.0694;
qf = [q1 q2 q3 q4 q5 q6];
qf = [1.0694 0.0637 -0.9054 0.0000 0.8417 -1.0694]; %INPUT

%Force and Moment Acting on the End Effector w.r.t base {0} coordinate
%system
f06 = [1;1;1];
n06 = [0.1;0.1;0.1];

%Z Axis Vector
Z = [0;0;1];


%Dynamic Analysis
w00 = [0;0;0]; %Angular Velocity of Base is Zero
v00 = [0;0;0]; %Linear Velocity of Base is Zero
dv00 = [0;0;0]; %Linear Acceleration of Base is Zero
dw00=[0;0;0]; %Angular Acceleration of Base is Zero

g = [0;9.81;0]; %Gravity Vector
m = 15; %Motor Mass
M = 10; %Mass of Each Link

%Inertial Tensor related to centre of mass of each link (Assumed to be zero
%if we assume each link as a point mass
IC11 = zeros(3);
IC22 = zeros(3);
IC33 = zeros(3);
IC44 = zeros(3);
IC55 = zeros(3);
IC66 = zeros(3);

%Cubic Trajectory(q = a*t^3+b*t^2+c*t^1+d)
%qi and qf can also be found from inverse kinematics when position of end
%effector is given(using Transformation matrix), as far as now qi=q0 and qf are already given readymade
%The trajectory can be plotted using command in PUMA560_1_forw_inv_traj

%Calculation of trajectory coefficitents(a,b,c,d)
%Assume dq at initial time and final time t0, tf as zero
tf = 5; %final time
a = -2*(qf.'-qi.')/(tf^3);
b = 3*(qf.'-qi.')/(tf^2);
c = zeros(length(a),1);
d = qi.';

%NEWTON-EULER RECURSIVE FORMULATION
k=0;
for t=0:0.05:tf
    k=k+1;
    q = a*t^3+b*t^2+c*t^1+d;
    dq = 3*a*t^2+2*b*t+c;
    ddq = 6*a*t+2*b;
    
    %Joint Variable
    Q1 = q(1);
    Q2 = q(2);
    Q3 = q(3);
    Q4 = q(4);
    Q5 = q(5);
    Q6 = q(6);
    
    %Joint Velocity
    dQ1 = dq(1);
    dQ2 = dq(2);
    dQ3 = dq(3);
    dQ4 = dq(4);
    dQ5 = dq(5);
    dQ6 = dq(6);
    
    %Joint Acceleration
    ddQ1 = ddq(1);
    ddQ2 = ddq(2);
    ddQ3 = ddq(3);
    ddQ4 = ddq(4);
    ddQ5 = ddq(5);
    ddQ6 = ddq(6);
    
    %Transformation Matrices & Rotation Matrices & Position Vectors
    T01 = trotz(Q1)*transl(0,0,0)*trotx(-pi/2);
    R01 = tr2rt(T01);
    R10 = R01.';
    O01 = transl(T01);
    T12 = trotz(Q2)*transl(a2,0,0)*trotx(0);
    R12 = tr2rt(T12);
    R21 = R12.';
    O12 = transl(T12);
    T23 = trotz(Q3)*transl(a3,0,d3)*trotx(-pi/2);
    R23 = tr2rt(T23);
    R32 = R23.';
    O23 = transl(T23);
    T34 = trotz(Q4)*transl(0,0,d4)*trotx(pi/2);
    R34 = tr2rt(T34);
    R43 = R34.';
    O34 = transl(T34);
    T45 = trotz(Q5)*transl(0,0,0)*trotx(-pi/2);
    R45 = tr2rt(T45);
    R54 = R45.';
    O45 = transl(T45);
    T56 = trotz(Q6)*transl(0,0,0)*trotx(0);
    R56 = tr2rt(T56);
    R65 = R56.';
    O56 = transl(T56);
    
    R06 = R01*R12*R23*R34*R45*R56;
    R60 = R10*R21*R32*R43*R54*R65;
    
    %Force and Moment acting on the endeffector in its own cooordinate system
    f66 = inv(R06)*f06;
    n66 = inv(R06)*n06;

    
    %Position of Centre of Mass of Each Link
    PC11 = O01/2;
    PC22 = O12/2;
    PC33 = O23/2;
    PC44 = O34/2;
    PC55 = O45/2;
    PC66 = O56/2;
    
    %Recursive Newton-Euler Formulation
    %Outward Iteration
    %for link1, i = 0
    w11 = R10*w00 + dQ1*Z;
    dw11 = R10*dw00 + R10*cross(w00,(dQ1*Z))+ddQ1*Z;
    v11 = R10*(v00+cross(w00,O01));
    dv11 = R10*(cross(w00,O01)+cross(w00,cross(w00,O01))+dv00+g);
    dvc1 = cross(dw11,PC11)+cross(w11,cross(w11,PC11))+dv11;
    F11 = M*dvc1;
    N11 = IC11*dw11+cross(w11,(IC11*w11));
    
    %for link2, i = 1
    w22 = R21*w11 + dQ2*Z;
    dw22 = R21*dw11 + R21*cross(w11,(dQ2*Z))+ddQ2*Z;
    v22 = R21*(v11+cross(w11,O12));
    dv22 = R21*(cross(w11,O12)+cross(w11,cross(w11,O12))+dv11+g);
    dvc2 = cross(dw22,PC22)+cross(w22,cross(w22,PC22))+dv22;
    F22 = M*dvc2;
    N22 = IC22*dw22+cross(w22,(IC22*w22));
    
    %for link3, i = 2
    w33 = R32*w22 + dQ3*Z;
    dw33 = R32*dw22 + R32*cross(w22,(dQ3*Z))+ddQ3*Z;
    v33 = R32*(v22+cross(w22,O23));
    dv33 = R32*(cross(w22,O23)+cross(w22,cross(w22,O23))+dv22+g);
    dvc3 = cross(dw33,PC33)+cross(w33,cross(w33,PC33))+dv33;
    F33 = M*dvc3;
    N33 = IC33*dw33+cross(w33,(IC33*w33));
    
    %for link4, i = 3
    w44 = R43*w33 + dQ4*Z;
    dw44 = R43*dw33 + R43*cross(w33,(dQ4*Z))+ddQ4*Z;
    v44 = R43*(v33+cross(w33,O34));
    dv44 = R43*(cross(w33,O34)+cross(w33,cross(w33,O34))+dv33+g);
    dvc4 = cross(dw44,PC44)+cross(w44,cross(w44,PC44))+dv44;
    F44 = M*dv44;
    N44 = IC44*dw44+cross(w44,(IC44*w44));
    
    %for link5, i = 4
    w55 = R54*w44 + dQ5*Z;
    dw55 = R54*dw44 + R54*cross(w44,(dQ5*Z))+ddQ5*Z;
    v55 = R54*(v44+cross(w44,O45));
    dv55 = R54*(cross(w44,O45)+cross(w44,cross(w44,O45))+dv44+g);
    dvc5 = cross(dw55,PC55)+cross(w55,cross(w55,PC55))+dv55;
    F55 = M*dv55;
    N55 = IC55*dw55+cross(w55,(IC55*w55));
    
    %for link6, i = 5
    w66 = R65*w55 + dQ6*Z;
    dw66 = R65*dw55 + R65*cross(w55,(dQ6*Z))+ddQ6*Z;
    v66 = R65*(v55+cross(w55,O56));
    dv66 = R65*(cross(w55,O56)+cross(w55,cross(w55,O56))+dv55+g);
    dvc6 = cross(dw66,PC66)+cross(w66,cross(w66,PC66))+dv66;
    F66 = M*dv66;
    N66 = IC66*dw66+cross(w66,(IC66*w66));
    
    
    %Inward Iteration
    %for link6, i =5
    f55 = R56*f66+F66;
    n55 = N66+R56*n66+cross(PC66,F66)+cross(O56,(R56*f66));
    tau55 = n55.'*Z;
    
    %for link5, i =4
    f44 = R45*f55+F55;
    n44 = N55+R45*n55+cross(PC55,F55)+cross(O45,(R45*f55));
    tau44 = n44.'*Z;
    
    %for link4, i =3
    f33 = R34*f44+F44;
    n33 = N44+R34*n44+cross(PC44,F44)+cross(O34,(R34*f44));
    tau33 = n33.'*Z;
    
    %for link3, i =2
    f22 = R23*f33+F33;
    n22 = N33+R23*n33+cross(PC33,F33)+cross(O23,(R23*f33));
    tau22 = n22.'*Z;
    
    %for link2, i =1
    f11 = R12*f22+F22;
    n11 = N22+R12*n22+cross(PC22,F22)+cross(O12,(R12*f22));
    tau11 = n11.'*Z;
    
    %for link1, i =0
    f00 = R01*f11+F11;
    n00 = N11+R01*n11+cross(PC11,F11)+cross(O01,(R01*f11));
    tau00 = n00.'*Z;
    
    %Joint TORQUES
    TAU1(k) = tau00;
    TAU2(k) = tau11;
    TAU3(k) = tau22;
    TAU4(k) = tau33;
    TAU5(k) = tau44;
    TAU6(k) = tau55;
end

figure;
t = 0:0.05:tf;
plot(t,TAU1,'r',t,TAU2,'b',t,TAU3,'g',t,TAU4,'y',t,TAU5,'c',t,TAU6,'m');
grid on;
xlabel('time (seconds)');
ylabel('Torque(N.m)');
legend('\tau1','\tau2','\tau3','\tau4','\tau5','\tau6');
