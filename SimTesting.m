clc
clear
close all

%% Testing out some of our class stuff

q0 = [0;0;0;0;0;0];
quad = Quadcopter(q0);
u = [9.9 10.1 10 10.1];
[tout,qout,quad] = simRotDynamics(quad,u,[0 10]);

figure;
showQuad(quad,100);
for i = 1:numel(quad.tAnim)
 showQuad(quad,i);
 i
end
qout