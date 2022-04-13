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


%Plotting Workspace of the Robot
conv = pi/180; %(degree to radian Conversion)
i = 0;
for q1 = deg2rad(-160):deg2rad(10):deg2rad(160)
    for q2 = deg2rad(-225):deg2rad(10):deg2rad(45)
        for q3 = deg2rad(-45):deg2rad(10):deg2rad(225)
%             for q4 = deg2rad(-110):deg2rad(10):deg2rad(110)
%                 for q5 = deg2rad(-100):deg2rad(10):deg2rad(100)
%                     for q6 = deg2rad(-266):deg2rad(10):deg2rad(266)
                        T01 = trotz(q1)*transl(0,0,0)*trotx(-pi/2);
                        T12 = trotz(q2)*transl(a2,0,0)*trotx(0);
                        T23 = trotz(q3)*transl(a3,0,d3)*trotx(-pi/2);
%                         T34 = trotz(q4)*transl(0,0,d4)*trotx(pi/2);
%                         T45 = trotz(q5)*transl(0,0,0)*trotx(-pi/2);
%                         T56 = trotz(q6)*transl(0,0,0)*trotx(0);
                        
                        T06 = T01*T12*T23; %*T34*T45*T56;
                        
                        i = i+1;
                        
                        P = T06(1:3,4);
                        P1(i) = P(1);
                        P2(i) = P(2);
                        P3(i) = P(3);
                    end
                end
            end
%         end
%     end
% end

figure;
plot3(P1,P2,P3,'b.');
