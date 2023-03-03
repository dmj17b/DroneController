clc
clear
close all

%% Testing out some of our class stuff
% q = [w1;w2;w3;w4;r;p;ya;dr;dp;dya];
q0 = [0;0;0;0;0;0;0;0;0;0];
quad = Quadcopter(q0);
u = [.01 0 .01 0];

% Call function to simulate quad dynamics based on 4 motor torques
[tout,qout,quad] = simRotDynamics(quad,u,[0 30]);



qout