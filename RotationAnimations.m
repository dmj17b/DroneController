clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;zdr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);

% Control Inputs:
u = [1.5 1 0.5 1];    % Example input for pitch
% u = [1 1.5 1 0.5];    % Example input for roll
% u = [1.5 1 1.5 1];    % Example input for yaw


% Call function to simulate quad dynamics based on 4 motor torques
[tout,qout,quad] = simDynamics(quad,u,[0 10]);


% Call function to plot roll, pitch, and yaw over time
figure;
plotStates(quad);

% Call function to generate animation (add file name to 4th input if you
% want to save the video as an MP4)
figure;
genRotAnim(quad,tout,qout,'Pitch Motion',"Pitch.mp4");