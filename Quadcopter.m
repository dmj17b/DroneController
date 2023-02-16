classdef Quadcopter
    
    
    properties
        %All states and accelerations:
        x,y,z;
        dx,dy,dz;
        ddx,ddy,ddz;
        r,p,ya;
        dr,dp,dya;
        ddr,ddp,ddya;

        %Properties of the quadcopter
        l = 0.2;    %Distance from each rotor to the CG in meters
        g = 9.81;   %Gravity in m/s^2
        m = 1.2;    %Some ambiguous mass in kg
        Ixx = 1;    %Inertia about x axis
        Iyy = 1;    %Inertia about y axis
        Izz = 1.3;  %Inertia about z axis

        %Visualization
        quadstl;

        %Change
    end
    
%% The methods section is where we will define any functions related to the quadcopter
    methods
        function quad = Quadcopter(x0)
            %This initializes the quadcopter object at some initial state
            %Also does any other intial math stuff to set up the quadcopter
            quad.quadstl = stlread("Benchy.stl");

        end
        
        function  quad = simDynamics(quad,u)
            % This function will simulate the dynamics of the quadcopter

        end

        function quad = showQuad(quad)
            cla
            trimesh(quad.quadstl)
        end
    end
end

