clear all;
close all;

%Trajectory
q11 = 160; q22 = 25; q33 = -50; q44 = 10; q55 = 45; q66 = -60;
qf1 = [q11 q22 q33 q44 q55 q66];
q0 = [0 0 0 0 0 0];
tf = 10;
t = 0:0.15:tf; %Time 0 to 5 seconds in the increment of 0.15 INPUT
Q = jtraj(q0,qf1,t); %Create Joint Angles from initial to final joint configuration


% %To show the trajectory of the robot
for i = 1:1:length(t)
   q1 = Q(i,1);
   q2 = Q(i,2);
   q3 = Q(i,3);
   q4 = Q(i,4);
   q5 = Q(i,5);
   q6 = Q(i,6);
   sim('PUMA560_Simulink')
end
