clc
clear
close all

%% Figuring out roll, pitch, yaw rotation matrices
syms r p y

% Rotation about the x axis (roll)
Rr = [1    0      0;
      0 cos(r) -sin(r);
      0 sin(r)  cos(r)];

% Rotation about the y axis (pitch)
Rp = [cos(p) 0 sin(p);
        0    1    0   ;
      -sin(p) 0 cos(p)];

% Rotation about the z axis (yaw)
Ry = [cos(y) -sin(y) 0;
      sin(y)  cos(y) 0;
        0      0     1];

Rf = Ry*Rp*Rr