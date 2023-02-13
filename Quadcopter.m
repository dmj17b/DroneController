classdef Quadcopter
    %QUAD Summary of this class goes here
    %   Detailed explanation goes here
    
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


    end
    
    methods
        function quad = Quadcopter(inputArg1,inputArg2)
            %This initializes the quadcopter object at some initial state
            %Also does any other intial math stuff to set up the quadcopter


        end
        
        function  quad = simDynamics(quad,u)
            %
        end
    end
end

