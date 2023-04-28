clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [196.2544;196.2544;196.2544;196.2544;0;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;z;dr;dp;dya;dx;dy;dz];
q0 = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]
quad = Quadcopter(q0);

Tf = 5;

% Control Inputs:
% u = 50*[1 1.5 1 0.5];    % Example input for pitch
% u = 50*[1 1.5 1 0.5];    % Example input for roll
% u = [1.5 1 1.5 1];    % Example input for yaw

u = 0.0044*[1 1 1 1];
uStar = 0.0044*[1 1 1 1]';
qStar = q0;

[A,B] = linearize3D(quad,qStar,uStar);
poles = linspace(-1, -100, 16);

K1 = zeros([4 16]);
K = place(A,B,poles);

% Call function to simulate quad dynamics based on 4 motor torques
% [tout,qout,quad] = simDynamics(quad,u,[0 Tf]);

[tout,qout] = ode45(@(t,q) quadODE(t,q,-K1*(q-qDes),quad),[0 Tf], q0);
quad.tSim = tout;
quad.qSim = qout;
% Call function to plot roll, pitch, and yaw over time
plotStates(quad);


%% Animation loop
FPS = 30;
tAnim = 0:1/FPS:Tf;
qAnim = interp1(tout,qout,tAnim);
figure

for i = 1:numel(tAnim)
    view([45 45])

    showQuad(quad,qAnim(i,:));
    title('ISO View')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    F1 = quad.kf*qAnim(i,1)^2;
    Fv1 = Rotate(qAnim(i,5),qAnim(i,6),qAnim(i,7))*[0;0;1];

    drawnow

end

