clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [196.2544;196.2544;196.2544;196.2544;pi/6;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;z;dr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);

Tf = 5;

% Control Inputs:
% u = 50*[1 1.5 1 0.5];    % Example input for pitch
% u = 50*[1 1.5 1 0.5];    % Example input for roll
% u = [1.5 1 1.5 1];    % Example input for yaw

u = 0.0044*[1 1 1 1];
uStar = 0.0044*[1 1 1 1]';



%% Test Linearization
% [A, B, statesFP, inputsFP] = linearizeDynamics(quad);


% testLin = Quadcopter([statesFP(1:7); zeros(3,1); statesFP(8:end); zeros(3,1)]);
% [tout,qout,testLin] = simDynamics(testLin,inputsFP,[0 Tf]);

qStar = [196.2544;196.2544;196.2544;196.2544;0;0;0;0;0;0;]
uStar = 0.0044*[1 1 1 1]';

[A,B] = linearizeRot(quad,qStar,uStar)
Ctr = ctrb(A,B);
poles = linspace(-1,-10,10)

K = place(A,B,poles);

rank(Ctr)

[tout,qout] = ode45(@(t,q) quadRotODE(t,q,K*q,quad),[0 5],quad.q0rot);

subplot(3,1,1)
plot(tout,qout(:,5))
xlabel("Time (sec)")
ylabel("Roll (rad)")

subplot(3,1,2)
plot(tout,qout(:,6))
xlabel("Time (sec)")
ylabel("Roll (rad)")

subplot(3,1,3)
plot(tout,qout(:,7))
xlabel("Time (sec)")
ylabel("Roll (rad)")

