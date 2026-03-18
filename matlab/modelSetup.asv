% FlexDrive mechanical model - torque input
% Input:  u = M (torque) [N*m]
% Outputs: y = [phi1; phi2]
% States:  x = [phi1; dphi1; phi2; dphi2]

clear; clc;

%% PARAMETERS

% Inertias (converted from g*m^2 to kg*m^2)
J1 = 0.612e-3;
J2 = 0.0902e-3;
J3 = 1.0e-6;      % small nonzero value (important!)

JL = J1 + J2;
JD = J3;

% Mechanical parameters (you will tune/identify these)
k  = 0.05;        % [N*m/rad]
B1 = 1e-4;        % [N*m*s/rad]
B2 = 1e-4;        % [N*m*s/rad]

%% STATE-SPACE MATRICES

A = [ 0        1         0        0;
     -k/JD   -B2/JD     k/JD      0;
      0        0         0        1;
      k/JL     0       -k/JL   -B1/JL ];

B = [0;
     1/JD;     % <-- torque input directly
     0;
     0];

C = [1 0 0 0;    % phi1
     0 0 1 0];   % phi2

D = [0;
     0];

Kpwm_M=1.925e-5;