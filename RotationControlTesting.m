clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [196.2544;196.2544;196.2544;196.2544;pi/6;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;z;dr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);

Tf = 1;


%% Linearization
% Fixed Point:
Freq = 0.25*(quad.m*quad.g);
omegaReq = sqrt(Freq/quad.kf);

qStar = [0;0;0;0;0;0;];
uStar = omegaReq*[1 1 1 1]';
qDes = [pi/36; 0;0;0;0;0];

% Linearize:
[A,B] = linearizeRot(quad,qStar,uStar)
Ctr = ctrb(A,B);
rank(Ctr)


% Pole placement:
poles = linspace(-10,-60,6)
K = place(A,B,poles)

% Simulate:
[tout,qout] = ode45(@(t,q) quadRotODE(t,q,K*q,quad),[0 5],[0;0;0;0;0;0]);

subplot(3,1,1)
plot(tout,qout(:,1))
xlabel("Time (sec)")
ylabel("Roll (rad)")

subplot(3,1,2)
plot(tout,qout(:,2))
xlabel("Time (sec)")
ylabel("Pitch (rad)")

subplot(3,1,3)
plot(tout,qout(:,3))
xlabel("Time (sec)")
ylabel("Yaw (rad)")

