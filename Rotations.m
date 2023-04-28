clc
clear
close all

%% Figuring out roll, pitch, yaw rotation matrices
syms r p ya x y z

% Rotation about the x axis (roll)
Rr = [1    0      0;
      0 cos(r) -sin(r);
      0 sin(r)  cos(r)];

% Rotation about the y axis (pitch)
Rp = [cos(p) 0 sin(p);
        0    1    0   ;
      -sin(p) 0 cos(p)];

% Rotation about the z axis (yaw)
Rya = [cos(ya) -sin(ya) 0;
      sin(ya)  cos(ya) 0;
        0      0     1];

Rf = Rya*Rp*Rr;

T = [Rf,[x;y;z];[0 0 0 1]];

% matlabFunction(Rf,'file','Rotate.m','vars',{r,p,ya})
matlabFunction(T,'file','TransRot.m','vars',{r,p,ya,x,y,z})