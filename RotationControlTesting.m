clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [196.2544;196.2544;196.2544;196.2544;pi/6;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;z;dr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);

Tf = 10;


%% Linearization
% Fixed Point:
Freq = 0.25*(quad.m*quad.g);
omegaReq = sqrt(Freq/quad.kf);

qStar = [0;0;0;0;0;0];
uStar = omegaReq*[1 1 1 1]';
qDes = [0; 0; 0; 0;0;0];
qi = [0;0;0.12;0;0;0];

% Linearize:
[A,B] = linearizeRot(quad,qStar,uStar);
Ctr = ctrb(A,B);
rank(Ctr)


% Pole placement:
% poles = linspace(-1,-6,6);
% poles = [-1 -2 -3 -1 -2 -3];
poles = [-2 -2 -2 -1 -1 -1];
K = place(A,B,poles);

% Simulate:
% options = odeset('RelTol',1000,'AbsTol',10);
[tout,qout] = ode45(@(t,q) quadRotODE(t,q,K*(qDes + qStar - q) + uStar,quad),[0 Tf],qi);

% h = 0.0001;
% tEuler = 0:h:Tf;
% q0 = qi;
% dq = quadRotODE(0, q0, K*(qDes - qStar - q0) + uStar, quad);
% qEuler = zeros(numel(tEuler), 6);
% accels = zeros(numel(tEuler), 3);
% 
% for iter = 1:numel(tEuler)
%     qEuler(iter, :) = q0;
%     accels(iter, :) = dq(4:end);
%     q0 = quadRotODE(0, q0, K*(qDes - qStar - q0) + uStar, quad).*h + q0;
%     dq = quadRotODE(0, q0, K*(qDes - qStar - q0) + uStar, quad);
% end
% 
% sys = ss(A-B*K,0*B,eye(6),[0]);
% 
% [qLin,tLin] = initial(sys,qi,Tf);


figure;
subplot(3,1,1)
sgtitle("Nonlinear Simulation")
plot(tout,qout(:,1))
% ylim([-1 1]);
xlabel("Time (sec)")
ylabel("Roll (rad)")

subplot(3,1,2)
plot(tout,qout(:,2))
% ylim([-1 1]);
xlabel("Time (sec)")
ylabel("Pitch (rad)")

subplot(3,1,3)
plot(tout,qout(:,3))
% ylim([-1 1]);
xlabel("Time (sec)")
ylabel("Yaw (rad)")

% figure();
% subplot(3,1,1);
% sgtitle("Euler Simulation")
% plot(tEuler,qEuler(:,1))
% xlabel("Time (sec)")
% ylabel("Roll (rad)")
% 
% subplot(3,1,2)
% plot(tEuler,qEuler(:,2))
% xlabel("Time (sec)")
% ylabel("Pitch (rad)")
% 
% subplot(3,1,3)
% plot(tEuler,qEuler(:,3))
% xlabel("Time (sec)")
% ylabel("Yaw (rad)")
% 
% figure()
% sgtitle("Euler Accelerations");
% subplot(3,1,1);
% plot(tEuler, accels(:,1));
% xlabel("Time (sec)")
% ylabel("Roll Acceleration (rad)")
% 
% subplot(3,1,2);
% plot(tEuler, accels(:,2));
% xlabel("Time (sec)")
% ylabel("Pitch Acceleration (rad)")
% 
% subplot(3,1,3);
% plot(tEuler, accels(:,3));
% xlabel("Time (sec)")
% ylabel("Yaw Acceleration (rad)")
% 
% 
% figure;
% sgtitle("Linear Simulation")
% subplot(3,1,1)
% plot(tLin,qLin(:,1))
% xlabel("Time (sec)")
% ylabel("Roll (rad)")
% 
% subplot(3,1,2)
% plot(tLin,qLin(:,2))
% xlabel("Time (sec)")
% ylabel("Pitch (rad)")
% 
% subplot(3,1,3)
% plot(tLin,qLin(:,3))
% xlabel("Time (sec)")
% ylabel("Yaw (rad)")

