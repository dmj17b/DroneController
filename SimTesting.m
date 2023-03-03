clc
clear
close all

%% Testing out some of our class stuff
% q = [w1;w2;w3;w4;r;p;ya;dr;dp;dya];
q0 = [0;0;0;0;0;0;0;0;0;0];
quad = Quadcopter(q0);
u = [1 1 1 1];
[tout,qout,quad] = simRotDynamics(quad,u,[0 10]);

figure;

for i = 1:numel(quad.tAnim)
 showQuad(quad,i);
 i
end
qout