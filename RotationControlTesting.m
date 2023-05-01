clc
clear
close all


%% Setup:
q0 = [196.2569;196.2569;196.2569;196.2569;pi/6;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;z;dr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);
Tf = 5;

%% Linearization and control:
[A, B, statesFP, inputsFP] = linearizeDynamics(quad);

testLin = Quadcopter([statesFP(1:7); zeros(3,1); statesFP(8:end); zeros(3,1)]);
% [tout,qout,testLin] = simDynamics(testLin,inputsFP,[0 Tf]);

figure();
% plotStates(testLin);

Ctr = ctrb(A,B);

rank(Ctr)


poles = -1:-1:-10;
K = place(A,B,poles);


%% Simulation:

[tout,qout] = ode45(@(t,q) quadRotODE(t,q,K*q,quad),[0 1],quad.q0rot);
quad.tSim = tout;
quad.qSim = qout;
plotStates(quad)


