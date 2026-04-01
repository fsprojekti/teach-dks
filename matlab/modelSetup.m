% FlexDrive mechanical model - torque input
% Input:  u = M (torque) [N*m]
% Outputs: y = [phi1 (deg); phi2 (deg); omega1 (rpm); omega2 (rpm)]
% States:  x = [phi1; dphi1; phi2; dphi2]

clear; clc;

%% PARAMETERS

% Inertias [kg*m^2] -- identified by GA
J1 = 1.7830e-4;
J2 = 7.6620e-5;
J3 = 4.2527e-6;

JL = J1 + J2;
JD = J3;

% Mechanical parameters -- identified by GA (ISE = 5.83e7)
k  = 9.6660;      % [N*m/rad]    spring stiffness
B1 = 1.9318e-4;   % [N*m*s/rad]  load-side damping
B2 = 1.0787e-4;   % [N*m*s/rad]  drive-side damping

%% STATE-SPACE MATRICES

A = [ 0        1         0        0;
     -k/JD   -B2/JD     k/JD      0;
      0        0         0        1;
      k/JL     0       -k/JL   -B1/JL ];

B = [0;
     1/JD;     % <-- torque input directly
     0;
     0];

C = [180/pi  0       0        0;       % phi1   [deg]
     0       0       180/pi   0;       % phi2   [deg]
     0       30/pi   0        0;       % omega1 [rpm]
     0       0       0        30/pi];  % omega2 [rpm]

D = [0; 0; 0; 0];

pwm = 2000;