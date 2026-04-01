% GA optimisation for FlexDrive state-space parameters
% Optimises: J1, J2, J3, k, B1, B2
% Data file: data.mat  columns: [M, phi1_deg, phi2_deg, omega1_rpm, omega2_rpm]

clear; clc;

%% Load measurement data
data = load('data.mat');
fields = fieldnames(data);
d = data.(fields{1});   % works regardless of variable name inside .mat

t_data   = (0:size(d,1)-1)' * 0.01;   % assumes 10 ms sample time -- adjust if different
M_in     = d(:,1);
y_meas   = d(:,2:5);   % [phi1_deg, phi2_deg, omega1_rpm, omega2_rpm]

%% GA bounds
%        J1        J2        J3        k        B1        B2
lb = [1e-4     1e-5     1e-8     1e-4    1e-5    1e-5  ];
ub = [5e-3     5e-3     1e-4     10      1e-1    1e-1  ];

nvars = 6;

opts = optimoptions('ga', ...
    'PopulationSize',    100,  ...
    'MaxGenerations',    200,  ...
    'FunctionTolerance', 1e-8, ...
    'Display',           'iter', ...
    'UseParallel',       false);

fitFcn = @(x) flexDriveFitness(x, t_data, M_in, y_meas);

[x_opt, fval] = ga(fitFcn, nvars, [], [], [], [], lb, ub, [], opts);

%% Report results
fprintf('\n--- Optimised Parameters ---\n');
fprintf('J1 = %.4e  [kg*m^2]\n', x_opt(1));
fprintf('J2 = %.4e  [kg*m^2]\n', x_opt(2));
fprintf('J3 = %.4e  [kg*m^2]\n', x_opt(3));
fprintf('k  = %.4e  [N*m/rad]\n', x_opt(4));
fprintf('B1 = %.4e  [N*m*s/rad]\n', x_opt(5));
fprintf('B2 = %.4e  [N*m*s/rad]\n', x_opt(6));
fprintf('Fitness (ISE) = %.6f\n', fval);

%% Update modelSetup with found values
J1 = x_opt(1); J2 = x_opt(2); J3 = x_opt(3);
k  = x_opt(4); B1 = x_opt(5); B2 = x_opt(6);
save('gaResult.mat', 'J1','J2','J3','k','B1','B2');

%% Plot best solution vs measured data
y_sim = flexDriveSimulate(x_opt, t_data, M_in);

labels  = {'phi_1 [deg]', 'phi_2 [deg]', '\omega_1 [rpm]', '\omega_2 [rpm]'};

figure('Name', 'GA Best Solution vs Measured', 'NumberTitle', 'off');
for i = 1:4
    subplot(2,2,i);
    plot(t_data, y_meas(:,i), 'Color', [1 0.75 0], 'LineWidth', 1.2); hold on;
    plot(t_data, y_sim(:,i),  'b', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(labels{i});
    title(labels{i});
    legend('Measured', 'Model', 'Location', 'best');
    grid on;
end
sgtitle(sprintf('GA fit  |  ISE = %.3g  |  J1=%.3e  J2=%.3e  J3=%.3e  k=%.3g  B1=%.3e  B2=%.3e', ...
    fval, x_opt(1), x_opt(2), x_opt(3), x_opt(4), x_opt(5), x_opt(6)));

% -------------------------------------------------------------------------
function y_sim = flexDriveSimulate(x, t_data, M_in)
    J1 = x(1); J2 = x(2); J3 = x(3);
    k  = x(4); B1 = x(5); B2 = x(6);

    JL = J1 + J2;  JD = J3;

    A = [ 0        1         0        0;
         -k/JD   -B2/JD     k/JD      0;
          0        0         0        1;
          k/JL     0       -k/JL   -B1/JL ];
    B = [0; 1/JD; 0; 0];
    C = [180/pi  0       0        0;
         0       0       180/pi   0;
         0       30/pi   0        0;
         0       0       0        30/pi];
    D = zeros(4,1);

    try
        y_sim = lsim(ss(A,B,C,D), M_in, t_data);
    catch
        y_sim = zeros(length(t_data), 4);
    end
end

% -------------------------------------------------------------------------
function ise = flexDriveFitness(x, t_data, M_in, y_meas)
    y_sim = flexDriveSimulate(x, t_data, M_in);

    if all(y_sim(:) == 0)
        ise = 1e12;
        return;
    end

    % ISE: integral of squared errors, summed over all 4 outputs
    err = y_sim - y_meas;
    dt  = t_data(2) - t_data(1);
    ise = dt * sum(sum(err.^2));
end
