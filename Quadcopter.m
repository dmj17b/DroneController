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
            dq(1) = (u(1) - quad.kb*q(1)^2)/quad.Im;
            dq(2) = (u(2) - quad.kb*q(2)^2)/quad.Im;
            dq(3) = (u(3) - quad.kb*q(3)^2)/quad.Im;
            dq(4) = (u(4) - quad.kb*q(4)^2)/quad.Im;



            % Calculate thrust forced based on motor speed
            F1 = quad.kf*q(1)^2;
            F2 = quad.kf*q(2)^2;
            F3 = quad.kf*q(3)^2;
            F4 = quad.kf*q(4)^2;

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
            dq(1) = (u(1) - quad.kb*q(1)^2)/quad.Im;
            dq(2) = (u(2) - quad.kb*q(2)^2)/quad.Im;
            dq(3) = (u(3) - quad.kb*q(3)^2)/quad.Im;
            dq(4) = (u(4) - quad.kb*q(4)^2)/quad.Im;



            % Calculate thrust forced based on motor speed
            F1 = quad.kf*q(1)^2;
            F2 = quad.kf*q(2)^2;
            F3 = quad.kf*q(3)^2;
            F4 = quad.kf*q(4)^2;

            Fv1 = F1*Rotate(q(5),q(6),q(7))*[0;0;1];
            Fv2 = F2*Rotate(q(5),q(6),q(7))*[0;0;1];
            Fv3 = F3*Rotate(q(5),q(6),q(7))*[0;0;1];
            Fv4 = F4*Rotate(q(5),q(6),q(7))*[0;0;1];

            % r,p,ya velocities
            dq(5) = q(11);
            dq(6) = q(12);
            dq(7) = q(13);

            % x,y,z velocities
            dq(8) = q(14);
            dq(9) = q(15);
            dq(10) = q(16);

            % r,p,ya accelerations
            dq(11) = (quad.L*quad.kf/quad.Ixx)*(F2-F4) - quad.cdr*q(11)/quad.Ixx;
            dq(12) = (quad.L*quad.kf/quad.Iyy)*(F3-F1) - quad.cdr*q(12)/quad.Iyy;
            dq(13) = (T1+T3-T4-T2 - quad.cdya*q(13))/quad.Izz;

            % x,y,z accelerations (set to zero while we figure out
            % rotations)
            dq(14) = Fv1(1)+Fv2(1)+Fv3(1)+Fv4(1) - quad.cdr*q(14);
            dq(15) = Fv1(2)+Fv2(2)+Fv3(2)+Fv4(2) - quad.cdr*q(15);
            dq(16) = Fv1(3)+Fv2(3)+Fv3(3)+Fv4(3) - quad.cdr*q(16) - quad.m*quad.g;

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
            cla
            axlim = 3;
            h = makehgtform('zrotate',qi(7),'yrotate',qi(6),'xrotate',qi(5),'translate',qi(8),qi(9),qi(10));
%             h = TransRot(qi(5),qi(6),qi(7),qi(8),qi(9),qi(10));
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

       

        % Plots roll, pitch, yaw, x, y, z over time
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

            figure;
            sgtitle('Position over time')
            subplot(3,1,1)
            plot(quad.tSim,quad.qSim(:,8))
            title('X Position vs. Time')

            subplot(3,1,2)
            plot(quad.tSim,quad.qSim(:,9))
            title('Y Positon vs. Time')

            subplot(3,1,3)
            plot(quad.tSim,quad.qSim(:,10))
            title('Z Position vs. Time')


        end

        % Function that plots the motor velocities in a bar chart
        function quad = motorVelBarChart(quad,qi)
            bar(qi(1:4));
            ylim([0 1e6]);
            title('Motor Angular Velocities (Rad/s)')

            drawnow;
        end

        % Generates a pretty animation of quad rotations
        function genRotAnim(quad,t,q,titre,filename)
            %Titre = title of plot
            %filename is option, but if included will export an MP4 of the
            %animation

            FPS = 30;
            tAnim = 0:1/FPS:t(end);
            qAnim = interp1(t,q,tAnim);

            if(nargin>4)
                v = VideoWriter(filename,'MPEG-4');
                v.FrameRate = FPS;
                open(v);
            end

            for i = 1:numel(tAnim)
                % On first subplot, show quadcopter
                ax1 = subplot(1,2,1);
                sgtitle(titre);
                showQuad(quad,qAnim(i,:));
                view(ax1,[45 45]);
                title('ISO View')
                xlabel('X')
                ylabel('Y')
                zlabel('Z')


                % On 2nd subplot, show motor velocities
                ax2 = subplot(1,2,2);
                motorVelBarChart(quad,qAnim(i,:));
                if(nargin>4)
                writeVideo(v,getframe(gcf));
                end
            end
        end

        function [A,B] = linearizeRot(quad,qStar,uStar)
            q = sym("q",[10 1]);
            u = sym("u",[4 1]);
            
            % Motor torques for the given inputs
            T1 = u(1);
            T2 = u(2);
            T3 = u(3);
            T4 = u(4);

            % Prop/Motor Dynamics:
            dq(1) = (u(1) - quad.kb*q(1)^2)/quad.Im;
            dq(2) = (u(2) - quad.kb*q(2)^2)/quad.Im;
            dq(3) = (u(3) - quad.kb*q(3)^2)/quad.Im;
            dq(4) = (u(4) - quad.kb*q(4)^2)/quad.Im;



            % Calculate thrust forced based on motor speed
            F1 = quad.kf*q(1)^2;
            F2 = quad.kf*q(2)^2;
            F3 = quad.kf*q(3)^2;
            F4 = quad.kf*q(4)^2;


            % Angular Velocities:
            dq(5) = q(8);
            dq(6) = q(9);
            dq(7) = q(10);

            % Angular accelerations
            dq(8) = (quad.L*quad.kf/quad.Ixx)*(F2-F4) - quad.cdr*q(5)/quad.Ixx;
            dq(9) = (quad.L*quad.kf/quad.Iyy)*(F3-F1) - quad.cdr*q(6)/quad.Iyy;
            dq(10) = (T1+T3-T4-T2 - quad.cdya*q(7))/quad.Izz;

            dq = dq';


            A = jacobian(dq,q);
            B = jacobian(dq,u);

            A = double(subs(A,q,qStar));
            B = double(subs(B,u,uStar));
        end

        function [A, B, statesFP, inputsFP] = linearizeDynamics(quad)
            syms tau1 tau2 tau3 tau4 omega1 omega2 omega3 omega4 domega1 domega2 domega3 domega4 r p ya dr dp dya ddr ddp ddya
            
            % Thrust force from the four rotors
            F1 = omega1.*quad.L;
            F2 = omega2.*quad.L;
            F3 = omega3.*quad.L;
            F4 = omega4.*quad.L;

            % All the xdot equations
            eqn1 = (1/quad.Im).*(tau1 - quad.kb.*omega1.^2);
            eqn2 = (1/quad.Im).*(tau2 - quad.kb.*omega2.^2);
            eqn3 = (1/quad.Im).*(tau3 - quad.kb.*omega3.^2);
            eqn4 = (1/quad.Im).*(tau4 - quad.kb.*omega4.^2);
            eqn5 = dr;
            eqn6 = dp;
            eqn7 = dya;
            eqn8_lhs = ddr;
            eqn8_rhs = (1/quad.Ixx).*(F2.*quad.L - F4.*quad.L - quad.cdr.*dr);
            eqn9_lhs = ddp;
            eqn9_rhs = (1/quad.Iyy).*(F3.*quad.L - F1.*quad.L - quad.cdr.*dp);
            eqn10_lhs = ddya;
            eqn10_rhs = (1/quad.Izz).*(tau1 + tau3 - tau2 - tau4 - quad.cdya.*dya);

            states = [omega1; omega2; omega3; omega4; r; p; ya; dr; dp; dya];
            derivStates = [domega1; domega2; domega3; domega4; dr; dp; dya; ddr; ddp; ddya];
            inputs = [tau1; tau2; tau3; tau4];
            eqns_rhs = [eqn1; eqn2; eqn3; eqn4; eqn5; eqn6; eqn7; eqn8_rhs; eqn9_rhs; eqn10_rhs];

            A = jacobian(eqns_rhs, states);
            B = jacobian(eqns_rhs, inputs);

            omega = sqrt(quad.m.*quad.g./(4.*quad.kb));
            % I was trying to generalize but I think this would override
            % the symbolic variable and screw stuff up
%             omega2 = omega1;
%             omega3 = omega1;
%             omega4 = omega1;

            eqn1 = subs(eqn1, [derivStates; omega1], [zeros(length(derivStates), 1); omega]) == 0;
            eqn2 = subs(eqn2, [derivStates; omega2], [zeros(length(derivStates), 1); omega]) == 0;
            eqn3 = subs(eqn3, [derivStates; omega3], [zeros(length(derivStates), 1); omega]) == 0;
            eqn4 = subs(eqn4, [derivStates; omega4], [zeros(length(derivStates), 1); omega]) == 0;
            eqn5 = dr == 0;
            eqn6 = dp == 0;
            eqn7 = dya == 0;
            eqn8 = subs(eqn8_rhs, [omega2; omega4; dr], [omega; omega; 0]) == 0;
            eqn9 = subs(eqn9_rhs, [omega1; omega3; dp], [omega; omega; 0]) == 0;
            eqn10 = subs(eqn10_rhs, dya, 0) == 0;

            eqns = [eqn1; eqn2; eqn3; eqn4];

            soln = solve(eqns, inputs);

            tau1 = double(soln.tau1);
            tau2 = double(soln.tau2);
            tau3 = double(soln.tau3);
            tau4 = double(soln.tau4);

            inputsFP = [tau1; tau2; tau3; tau4];
            statesFP = [omega; omega; omega; omega; zeros(6,1)];

            debug = false;
            if (debug)
                eqn10 = subs(eqn10_rhs, [inputs; dya], [inputsNum; 0]);
            end

            A = double(subs(A, states, statesFP));
            B = double(subs(B, inputs, inputsFP));
        end
        

    end
end

