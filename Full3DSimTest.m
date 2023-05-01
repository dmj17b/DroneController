clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [196.2569;196.2569;196.2569;196.2569;0;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;z;dr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);
Tf = 1.5;

% Control Inputs:
u = [20 15 5 10];    % Example input for pitch
u = 50*[1 1.5 1 0.5];    % Example input for roll
% u = [1.5 1 1.5 1];    % Example input for yaw

u = 0.044*[1 1 1 1];


% Call function to simulate quad dynamics based on 4 motor torques
[tout,qout,quad] = simDynamics(quad,u,[0 Tf]);

% Call function to plot roll, pitch, and yaw over time
plotStates(quad);



% %% Dummy simple animation loop
% FPS = 30;
% tAnim = 0:1/FPS:Tf;
% qAnim = interp1(tout,qout,tAnim);
% 
% 
% for i = 1:numel(tAnim)
%     figure(2);
%     view([45 45])
% 
%     showQuad(quad,qAnim(i,:));
%     title('ISO View')
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')
% 
%     F1 = quad.kf*qAnim(i,1)^2;
%     Fv1 = Rotate(qAnim(i,5),qAnim(i,6),qAnim(i,7))*[0;0;1];
%     quiver3(0,0,0,Fv1(1),Fv1(2),Fv1(3));
%     drawnow;
% 
%     
% end

%% Test Linearization
[A, B, statesFP, inputsFP] = linearizeDynamics(quad);


testLin = Quadcopter([statesFP(1:7); zeros(3,1); statesFP(8:end); zeros(3,1)]);
[tout,qout,testLin] = simDynamics(testLin,inputsFP,[0 Tf]);

figure();
plotStates(testLin);

Ctr = ctrb(A,B);
poles = -1:-1:-10;

K = place(A,B,poles);

rank(Ctr);

[tout,qout] = ode45(@(t,q) quadRotODE(t,q,K*q,quad),[0 5],quad.q0)







