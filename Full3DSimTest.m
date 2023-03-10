clc
clear
close all

%% 

% Initializing the Quadcopter:
q0 = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0]; % q = [w1;w2;w3;w4;r;p;ya;x;y;zdr;dp;dya;dx;dy;dz];
quad = Quadcopter(q0);

% Control Inputs:
u = [1.5 1 0.5 1];    % Example input for pitch
% u = [1 1.5 1 0.5];    % Example input for roll
% u = [1.5 1 1.5 1];    % Example input for yaw


% Call function to simulate quad dynamics based on 4 motor torques
[tout,qout,quad] = simDynamics(quad,u,[0 5]);


% Call function to plot roll, pitch, and yaw over time
plotStates(quad);




%% Dummy simple animation loop

for i = 1:numel(tout)
    figure(2);
%     subplot(2,2,1)
    ax1 = subplot(1,2,1);
    view(ax1, [45 45])

    showQuad(quad,qout(i,:));
    title('ISO View')
    xlabel('X')
    ylabel('Y')
    zlabel('Z')

    subplot(1,2,2)
    motorVelBarChart(quad,qout(i,:));
    title('Motor Velocities in Rad/s')

%     subplot(2,2,2)
%     showQuad(quad,qout(i,:));
%     title('X-Plane')
%     view(0,0)
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')
% 
%     subplot(2,2,3)
%     showQuad(quad,qout(i,:));
%     title('Y-Plane')
%     view(90,0)
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')
% 
%     subplot(2,2,4)
%     showQuad(quad,qout(i,:));
%     title('Z-Plane')
%     view(0,90)
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')

%     figure(3)
%     motorVelChart(quad,qout(i,:));


end

