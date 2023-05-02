clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [196.2544;196.2544;196.2544;196.2544;pi/6;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;z;dr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);  % Initializing quadcopter (q0 doesnt matter rn)

Tf = 5; % Final sim time


%% Linearization
% Fixed Point:
Freq = 0.25*(quad.m*quad.g);
omegaReq = sqrt(Freq/quad.kf);

desThrust = quad.m*quad.g;  % Need a thrust value to solve for rotor velocities

qStar = [0;0;0;0;0;0;];     % Just linearizing about 0 angles
uStar = [0;0;0];
qDes = [0; pi/4; 0;0;0;0];

% Linearize:
[A,B] = linearizeRot(quad,qStar,uStar)
Ctr = ctrb(A,B);
rank(Ctr)


% Pole placement:
poles = linspace(-1,-6,6)
K = place(A,B,poles)

% Simulate:
[tout,qout] = ode45(@(t,q) quadRotODE(t,q,-K*(q+qStar-qDes)+uStar,quad),[0 Tf],[0;0;0;0;0;0]);

uout = -K*(qout'-qDes)+uStar;

w1 = sqrt((desThrust/4*quad.kf) - uout(2,:)/(2*quad.kf*quad.L) - uout(3,:)/(4*quad.kb));
w2 = sqrt((desThrust/4*quad.kf) - uout(1,:)/(2*quad.kf*quad.L) + uout(3,:)/(4*quad.kb));
w3 = sqrt((desThrust/4*quad.kf) + uout(2,:)/(2*quad.kf*quad.L) - uout(3,:)/(4*quad.kb));
w4 = sqrt((desThrust/4*quad.kf) + uout(1,:)/(2*quad.kf*quad.L) + uout(3,:)/(4*quad.kb));


% Plot states:
figure;
subplot(3,1,1)
sgtitle("Roll, Pitch, Yaw")
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


% Plotting motor velocities:
figure
subplot(4,1,1)
sgtitle("Motor Velocities")
plot(tout,w1)
xlabel("Time (sec)")
ylabel("M1 (rad/s)")


subplot(4,1,2)
plot(tout,w2)
xlabel("Time (sec)")
ylabel("M2 (rad/s)")


subplot(4,1,3)
plot(tout,w3)
xlabel("Time (sec)")
ylabel("M3 (rad/s)")


subplot(4,1,4)
plot(tout,w4)
xlabel("Time (sec)")
ylabel("M4 (rad/s)")

% Animate:
figure;
genRotAnim(quad,tout,qout,"Pitch Control","Animations/Pitch.mp4")