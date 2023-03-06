clc
clear
close all

%% 

% q = [w1;w2;w3;w4;r;p;ya;x;y;zdr;dp;dya;dx;dy;dz];
q0 = [0;0;0;0;0;0;0;0;0;0;0;0;0;0;0;0];
quad = Quadcopter(q0);
u = [1 1.1 1 1.1];

% Call function to simulate quad dynamics based on 4 motor torques
[tout,qout,quad] = simDynamics(quad,u,[0 5]);


% Dummy simple animation loop
for i = 1:numel(tout)
    showQuad(quad,qout(i,:))
end

