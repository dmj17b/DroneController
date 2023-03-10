classdef Quadcopter
    
    
    properties
        %All states and accelerations:
        x,y,z;
        dx,dy,dz;
        ddx,ddy,ddz;
        r,p,ya;
        dr,dp,dya;
        ddr,ddp,ddya;
        q0;
        q0rot;
        %Properties of the quadcopter
        L = 0.225;    %Distance from each rotor to the CG in meters
        g = 9.81;   %Gravity in m/s^2
        m = 0.468;    %Some ambiguous mass in kg
        Ixx = 4.856e-3;    %Inertia about x axis
        Iyy = 4.856e-3;    %Inertia about y axis
        Izz = 8.801e-3;  %Inertia about z axis
        Im = 3.357e-5;   %Motor/prop inertia

        kf = 2.98e-5;   %Lift coefficient for propeller rotation
        kb = 1.14e-7;      %Drag coefficient for propeller rotation
        cdr = 0.25;  %Drag coefficient for roll/pitch
        cdya = 0.25; %Drag coefficient for yaw

        %Outputs from sim/controls:
        qSimRot,tSimRot;    % Sim states when we're only looking at rotations
        qAnimRot,tAnimRot;  % Anim states when we're only looking at rotations

        qSim, tSim;         % Full 6dof state w/ prop speeds

    end
    
%% The methods section is where we will define any functions related to the quadcopter
    methods
        function quad = Quadcopter(x0)
            %This initializes the quadcopter object at some initial state
            %Also does any other intial math stuff to set up the quadcopter
            
            %      1    2   3   4  5  6  7   8  9  10 11  12   13  14  15  16
            % q = [w1; w2; w3; w4; r; p; ya; x; y; z; dr; dp; dya; dx; dy; dz];

            quad.q0 = x0;
            quad.q0rot = [x0(1:7);x0(11:13)];

        end
        
        % ODE function for strictly simulating rotations
        function  dq = quadRotODE(t,q,u,quad)
            % This is the ODE function that represents the dynamics of the
            % system
            %      1;  2; 3; 4;5;6;7; 8; 9; 10;
            % q = [w1;w2;w3;w4;r;p;ya;dr;dp;dya];

            T1 = u(1);
            T2 = u(2);
            T3 = u(3);
            T4 = u(4);

            % Prop/Motor Dynamics:
            dq(1) = u(1)/quad.Im - quad.kb*q(1);
            dq(2) = u(2)/quad.Im - quad.kb*q(2);
            dq(3) = u(3)/quad.Im - quad.kb*q(3);
            dq(4) = u(4)/quad.Im - quad.kb*q(4);


            % Calculate thrust forced based on motor speed
            F1 = quad.kf*dq(1)^2;
            F2 = quad.kf*dq(2)^2;
            F3 = quad.kf*dq(3)^2;
            F4 = quad.kf*dq(4)^2;

            % r,p,ya velocities
            dq(5) = q(8);
            dq(6) = q(9);
            dq(7) = q(10);

            % r,p,ya accelerations
            dq(8) = (quad.L*quad.kf/quad.Ixx)*(F2-F4) - quad.cdr*q(8);
            dq(9) = (quad.L*quad.kf/quad.Iyy)*(F3-F1) - quad.cdr*q(9);
            dq(10) = (T1+T3-T4-T2 - quad.cdya*q(10))/quad.Izz;

            dq = dq';
        end

        % Simulates rotational dynamics without considering x,y,z
        function [tout,qout,quad] = simRotDynamics(quad,u,tspan)
            %Basically just a wrapper function to call ode45 for simulation
            [tout,qout] = ode45(@(t,q) quadRotODE(t,q,u,quad),tspan,quad.q0rot);
            quad.tSimRot = tout;
            quad.qSimRot = qout;
        end


        % ODE function for the full quadcopter state (translation and rotation)
        function  dq = quadODE(t,q,u,quad)
            % This is the ODE function that represents the dynamics of the
            % system

            %      1    2   3   4  5  6  7   8  9  10 11  12   13  14  15  16
            % q = [w1; w2; w3; w4; r; p; ya; x; y; z; dr; dp; dya; dx; dy; dz];


            % Motor torques for the given inputs
            T1 = u(1);
            T2 = u(2);
            T3 = u(3);
            T4 = u(4);

            % Prop/Motor Dynamics:
            dq(1) = u(1)/quad.Im - quad.kb*q(1)^2;
            dq(2) = u(2)/quad.Im - quad.kb*q(2)^2;
            dq(3) = u(3)/quad.Im - quad.kb*q(3)^2;
            dq(4) = u(4)/quad.Im - quad.kb*q(4)^2;


            % Calculate thrust forced based on motor speed
            F1 = quad.kf*dq(1)^2;
            F2 = quad.kf*dq(2)^2;
            F3 = quad.kf*dq(3)^2;
            F4 = quad.kf*dq(4)^2;

            % r,p,ya velocities
            dq(5) = q(11);
            dq(6) = q(12);
            dq(7) = q(13);

            % r,p,ya accelerations
            dq(11) = (quad.L*quad.kf/quad.Ixx)*(F2-F4) - quad.cdr*q(11)/quad.Ixx;
            dq(12) = (quad.L*quad.kf/quad.Iyy)*(F3-F1) - quad.cdr*q(12)/quad.Iyy;
            dq(13) = (T1+T3-T4-T2 - quad.cdya*q(13))/quad.Izz;

            % x,y,z accelerations (set to zero while we figure out
            % rotations)
            dq(14) = 0;
            dq(15) = 0;
            dq(16) = 0;

            dq = dq';
        end

        function [tout,qout,quad] = simDynamics(quad,u,tspan)
            %Basically just a wrapper function to call ode45 for simulation
            [tout,qout] = ode45(@(t,q) quadODE(t,q,u,quad),tspan,quad.q0);
            quad.tSim = tout;
            quad.qSim = qout;
        end




        % Function that plots the quadcopter given a certain position and
        % rotation
        function quad = showQuad(quad,qi)
            % This shows a certain frame of the quadcopter animation
            % state (frame i of tAnim, qAnim)
            axlim = 1;
            cla

            h = makehgtform('zrotate',qi(7),'yrotate',qi(6),'xrotate',qi(5));
            L1 = [-quad.L quad.L; 0 0; 0 0; 1 1];
            L2 = [0 0; -quad.L quad.L; 0 0; 1 1];
            L1t = h*L1;
            L2t = h*L2;
            hold on
            plot3(L1t(1,:),L1t(2,:),L1t(3,:),'Linewidth',3);
            plot3(L2t(1,:),L2t(2,:),L2t(3,:),'Linewidth',3);
            axis equal
            axis([-axlim, axlim, -axlim, axlim, -axlim, axlim]);
            drawnow;
        end

       

        % Plots roll, pitch, and yaw over time
        function quad = plotStates(quad)
            sgtitle('Angles over time')
            subplot(3,1,1)
            plot(quad.tSim,quad.qSim(:,5))
            title('Roll Angle vs. Time')

            subplot(3,1,2)
            plot(quad.tSim,quad.qSim(:,6))
            title('Pitch Angle vs. Time')

            subplot(3,1,3)
            plot(quad.tSim,quad.qSim(:,7))
            title('Yaw Angle vs. Time')
        end

        % Function that plots the motor velocities in a bar chart
        function quad = motorVelBarChart(quad,qi)
            bar(qi(1:4));
            ylim([0 1e6]);
            drawnow;
        end

    end
end

