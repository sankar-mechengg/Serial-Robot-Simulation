clc;    % Clear the command window.
close all;  % Close all figures (except those of imtool.)
imtool close all;  % Close all imtool figures if you have the Image Processing Toolbox.
clear;  % Erase all existing variables. Or clearvars if you want.
workspace;  % Make sure the workspace panel is showing.
format long g;
format compact;
fontSize = 36;
% Check that user has the Image Processing Toolbox installed.
hasIPT = license('test', 'image_toolbox');
if ~hasIPT
  % User does not have the toolbox installed.
  message = sprintf('Sorry, but you do not seem to have the Image Processing Toolbox.\nDo you want to try to continue anyway?');
  reply = questdlg(message, 'Toolbox missing', 'Yes', 'No', 'Yes');
  if strcmpi(reply, 'No')
    % User said No, so exit.
    return;
  end
end
%===============================================================================
% Read in a standard MATLAB color demo image.
folder = 'D:\Sync Drives\OneDrive - Indian Institute of Science\IISc PhD 2020\Course Work\2.Even 20-21\ME246 Robotics\Homework\Term Project\Robotics Project\My Robotics Project';
baseFileName = 'part1 - Copy.jpg';
% Get the full filename, with path prepended.
fullFileName = fullfile(folder, baseFileName);
if ~exist(fullFileName, 'file')
  % Didn't find it there.  Check the search path for it.
  fullFileName = baseFileName; % No path this time.
  if ~exist(fullFileName, 'file')
    % Still didn't find it.  Alert user.
    errorMessage = sprintf('Error: %s does not exist.', fullFileName);
    uiwait(warndlg(errorMessage));
    return;
  end
end
rgbImage = imread(fullFileName);
% Get the dimensions of the image.  numberOfColorBands should be = 3.
[rows, columns, numberOfColorBands] = size(rgbImage);
% Display the original color image.
subplot(2, 2, 1);
imshow(rgbImage);
axis on;
title('Original Color Image', 'FontSize', fontSize);
% Enlarge figure to full screen.
set(gcf, 'Units', 'Normalized', 'Outerposition', [0, 0, 1, 1]);
% Extract the individual red, green, and blue color channels.
% redChannel = rgbImage(:, :, 1);
greenChannel = rgbImage(:, :, 2);
% blueChannel = rgbImage(:, :, 3);
% Get the binaryImage
binaryImage = greenChannel < 200;
% Display the original color image.
subplot(2, 2, 2);
imshow(binaryImage);
axis on;
title('Binary Image', 'FontSize', fontSize);
% Find the baseline
verticalProfile  = sum(binaryImage, 2);
lastLine = find(verticalProfile, 1, 'last')
% Scan across columns finding where the top of the hump is
for col = 1 : columns
  yy = lastLine - find(binaryImage(:, col), 1, 'first');
  if isempty(yy)
    y(col) = 0;
  else
    y(col) = yy;
  end
  x(col) = col;
end
subplot(2, 2, 3);
plot(1 : columns, y, 'b-', 'LineWidth', 3);
grid on;
title('Y vs. X', 'FontSize', fontSize);
ylabel('Y', 'FontSize', fontSize);
xlabel('X', 'FontSize', fontSize);
x=x/1000;
y=y/1000;
x=x.';
y=y.';



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
Tf = PU.fkine(qrest);
Tf1 = PU.fkine(qrest);%Transformation matrix from Forward Kinematics



z=0.1;

for j = 1:length(x)
    if j == 1
        q1 = 5*x(j);
        k = (1/2*a2)*(x(j)^2+y(j)^2+z^2-d3^2-a2^2-a3^2-d4^2);
        q3 = 5*y(j);
        q2 = 4*x(j);
        
        qi = [0 0 0 0 0 0];
        qf = [q1 q2 q3 0 0 0];
    else
        q1 = 5*x(j-1);
        k = (1/2*a2)*(x(j)^2+y(j)^2+z^2-d3^2-a2^2-a3^2-d4^2);
        q3 = 5*y(j-1);
        q2 = 4*x(j-1);
        
        q10 = 5*x(j);
        k = (1/2*a2)*(x(j-1)^2+y(j-1)^2+z^2-d3^2-a2^2-a3^2-d4^2);
        q30 = 5*y(j);
        q20 = 4*x(j);
        
        qi = [q10 q20 q30 0 0 0];
        qf = [q1 q2 q3 0 0 0];
    end
    
    %Trajectory
    t = 0:0.25:0.5; %Time 0 to 5 seconds in the increment of 0.15 INPUT
    Q = jtraj(qi,qf,t); %Create Joint Angles from initial to final joint configuration
    Tr = PU.fkine(Q);
    
    %To show the trajectory of the robot
    for i = 1:1:length(t)
        T = Tr(i);
        trs = transl(T); %Takes the last column of Translation Matrix of T
        xx(i) = trs(1);
        yy(i) = trs(2);
        zz(i) = trs(3);
    end
    figure(2);
    plot3(xx,yy,zz,'color',[1 0 0],'LineWidth',2);
    hold on;
    plot(PU,Q);hold off; drawnow;
    
end


