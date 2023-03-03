classdef Quadcopter
    
    
    properties
        %All states and accelerations:
        x,y,z;
        dx,dy,dz;
        ddx,ddy,ddz;
        r,p,ya;
        dr,dp,dya;
        ddr,ddp,ddya;
        q0; qSim;

        %Properties of the quadcopter
        L = 0.225;    %Distance from each rotor to the CG in meters
        g = 9.81;   %Gravity in m/s^2
        m = 0.468;    %Some ambiguous mass in kg
        Ixx = 4.856e-3;    %Inertia about x axis
        Iyy = 4.856e-3;    %Inertia about y axis
        Izz = 8.801e-3;  %Inertia about z axis
        Im = 3.357e-5;   %Motor/prop inertia

        kf = 2.98e-6;   %Lift coefficient for propeller rotation
        kb = 1.14e-7;      %Drag coefficient for propeller rotation
        cdr = 0.25;  %Drag coefficient for roll/pitch
        cdya = 0.25; %Drag coefficient for yaw

        %Visualization
        quadstl;
        fstl,vstl;
        %Outputs from sim/controls:
        qsim,tsim;
        qAnim,tAnim;
    end
    
%% The methods section is where we will define any functions related to the quadcopter
    methods
        function quad = Quadcopter(x0)
            %This initializes the quadcopter object at some initial state
            %Also does any other intial math stuff to set up the quadcopter
            
            % x0 = [r, p, ya, dr, dp, dya];

            quad.quadstl = stlread("Benchy.stl");
%             [quad.vstl,quad.fstl]=stlread(quad.quadstl);
            quad.q0 = x0;

        end
        
        function  dq = quadRotODE(t,q,u,quad)
            % This is the ODE function that represents the dynamics of the
            % system

            T1 = u(1);
            T2 = u(2);
            T3 = u(3);
            T4 = u(4);

            F1 = quad.kf*T1^2;
            F2 = quad.kf*T2^2;
            F3 = quad.kf*T3^2;
            F4 = quad.kf*T4^2;

            dq(1) = q(4);
            dq(2) = q(5);
            dq(3) = q(6);

            dq(4) = (quad.L*quad.kf/quad.Ixx)*(F2-F4) - quad.cdr*q(4);
            dq(5) = (quad.L*quad.kf/quad.Iyy)*(F3-F1) - quad.cdr*q(5);
            dq(6) = (T1+T3-T4-T2 - quad.cdya*q(6))/quad.Izz;
            dq = dq';
        end

        function [tout,qout,quad] = simRotDynamics(quad,u,tspan)
            %Basically just a wrapper function to call ode45 for simulation
            [tout,qout] = ode45(@(t,q) quadRotODE(t,q,u,quad),tspan,quad.q0);
            quad.tAnim = tout;
            quad.qAnim = qout;
            quad.tsim = tout;
            quad.qsim = qout;

        end


        function quad = showQuad(quad,i)
            % This shows a certain frame of the quadcopter animation
            % state (frame i of tAnim, qAnim)
            figure(1);
            axlim = 1;
            cla
            L = quad.L;
            h = makehgtform('xrotate',quad.qAnim(i,1),'yrotate',quad.qAnim(i,2),'zrotate',quad.qAnim(i,3));
            L1 = [-L L; 0 0; 0 0; 1 1];
            L2 = [0 0; -L L; 0 0; 1 1];
            L1t = h*L1;
            L2t = h*L2;
            hold on
            plot3(L1t(1,:),L1t(2,:),L1t(3,:));
            plot3(L2t(1,:),L2t(2,:),L2t(3,:));
            axis equal
            axis([-axlim, axlim, -axlim, axlim, -axlim, axlim]);
            view(45,45)
            drawnow;
        end
    end
end

