% GA optimisation for FlexDrive state-space parameters
% Optimises: J1, J2, J3, k, B1, B2, Ku2M
% Data file: data.mat  columns: [PWM, phi1_deg, phi2_deg, omega1_rpm, omega2_rpm]

clear; clc;

PENALTY_BASE = 1e9;

%% Load measurement data
data = load('data.mat');
fields = fieldnames(data);
d = data.(fields{1});   % works regardless of variable name inside .mat

if ~isnumeric(d) || ndims(d) ~= 2
    error('data.mat must contain a numeric 2-D matrix.');
end

% If data was saved as 5xN instead of Nx5, transpose it.
if size(d,2) < 5 && size(d,1) >= 5
    d = d.';
end

if size(d,2) < 5
    error('Expected at least 5 columns: [PWM, phi1_deg, phi2_deg, omega1_rpm, omega2_rpm].');
end

d = double(d(:,1:5));
if any(~isfinite(d(:)))
    error('Measurement matrix contains NaN or Inf values.');
end
if size(d,1) < 3
    error('Measurement matrix has too few rows for simulation/fitness evaluation.');
end

Ts = 0.01;                            % assumes 10 ms sample time -- adjust if different
t_data   = (0:size(d,1)-1)' * Ts;
u_pwm    = d(:,1);
y_meas   = d(:,2:5);   % [phi1_deg, phi2_deg, omega1_rpm, omega2_rpm]

%% GA bounds
%        J1        J2        J3        k        B1        B2      Ku2M
lb = [1e-4     1e-5     1e-6     1e-3    1e-5    1e-5    1e-6  ];
ub = [5e-3     5e-3     1e-4     10      1e-1    1e-1    5e-3  ];
%
% Physical estimate for Ku2M (PWM -> output-shaft torque):
%   Chain:  PWM_max=4000  ->  driver duty=100%  ->  V_motor~12 V
%           R_motor ~ 7.5 Ohm  ->  I_stall = 12/7.5 = 1.6 A
%           Kt (typical small DC motor) ~ 15 mNm/A
%           Motor-shaft stall torque = 15e-3 * 1.6 = 24 mNm
%           Gear ratio N = 20.4  ->  output-shaft stall torque ~ 0.5 Nm
%           Ku2M_est = 0.5 Nm / 4000 PWM = 1.25e-4 Nm/PWM
% Upper bound: ub Ku2M = 5e-3 -> 20 Nm at full PWM (generous search room)
% Lower bound: lb Ku2M = 1e-6 -> practically zero gain

nvars = 7;
REG_WEIGHT = 0.05;

% Seed GA with one physically derived initial individual
Ku2M_est = 1.25e-4;
x0_phys  = [1e-3   5e-4   1e-5   1.0   1e-3   1e-3   Ku2M_est];

opts = optimoptions('ga', ...
    'PopulationSize',          100,  ...
    'MaxGenerations',          200,  ...
    'FunctionTolerance',       1e-8, ...
    'Display',                 'iter', ...
    'UseParallel',             false, ...
    'InitialPopulationMatrix', x0_phys);

fitFcn = @(x) flexDriveFitness(x, t_data, u_pwm, y_meas, lb, ub, x0_phys, REG_WEIGHT, PENALTY_BASE);

% Pre-GA diagnostic: verify simulation at physical seed point.
[~, seed_ok, seed_msg] = flexDriveSimulate(x0_phys, t_data, u_pwm);
if ~seed_ok
    error('Seed simulation failed before GA: %s', seed_msg);
end

[x_opt, fval] = ga(fitFcn, nvars, [], [], [], [], lb, ub, [], opts);

%% Report results
fprintf('\n--- Optimised Parameters ---\n');
fprintf('J1 = %.4e  [kg*m^2]\n', x_opt(1));
fprintf('J2 = %.4e  [kg*m^2]\n', x_opt(2));
fprintf('J3 = %.4e  [kg*m^2]\n', x_opt(3));
fprintf('k  = %.4e  [N*m/rad]\n', x_opt(4));
fprintf('B1 = %.4e  [N*m*s/rad]\n', x_opt(5));
fprintf('B2 = %.4e  [N*m*s/rad]\n', x_opt(6));
fprintf('Ku2M = %.4e  [N*m/PWM]\n', x_opt(7));
fprintf('Fitness (ISE) = %.6f\n', fval);

%% Update modelSetup with found values
J1 = x_opt(1); J2 = x_opt(2); J3 = x_opt(3);
k  = x_opt(4); B1 = x_opt(5); B2 = x_opt(6); Ku2M = x_opt(7);
save('gaResult.mat', 'J1','J2','J3','k','B1','B2','Ku2M');

%% Plot best solution vs measured data
[y_sim, sim_ok] = flexDriveSimulate(x_opt, t_data, u_pwm);

labels  = {'phi_1 [deg]', 'phi_2 [deg]', '\omega_1 [rpm]', '\omega_2 [rpm]'};

figure('Name', 'GA Best Solution vs Measured', 'NumberTitle', 'off');
for i = 1:4
    subplot(2,2,i);
    plot(t_data, y_meas(:,i), 'Color', [1 0.75 0], 'LineWidth', 1.2); hold on;
    if sim_ok
        plot(t_data, y_sim(:,i), 'b', 'LineWidth', 1.5);
        legend('Measured', 'Model', 'Location', 'best');
    else
        legend('Measured', 'Location', 'best');
    end
    xlabel('Time [s]');
    ylabel(labels{i});
    title(labels{i});
    grid on;
end
sgtitle(sprintf('GA fit  |  ISE = %.3g  |  J1=%.3e  J2=%.3e  J3=%.3e  k=%.3g  B1=%.3e  B2=%.3e  Ku2M=%.3e', ...
    fval, x_opt(1), x_opt(2), x_opt(3), x_opt(4), x_opt(5), x_opt(6), x_opt(7)));

% -------------------------------------------------------------------------
function [y_sim, ok, msg] = flexDriveSimulate(x, t_data, u_pwm)
    J1 = x(1); J2 = x(2); J3 = x(3);
    k  = x(4); B1 = x(5); B2 = x(6);
    Ku2M = x(7);

    msg = '';

    t_data = t_data(:);
    M_in = Ku2M * u_pwm(:);  % torque input from PWM

    JL = J1 + J2;  JD = J3;
    if ~all(isfinite(x)) || JL <= 0 || JD <= 0
        y_sim = [];
        ok = false;
        msg = 'non-finite parameters or non-positive inertia.';
        return;
    end

    if ~isfinite(k) || ~isfinite(B1) || ~isfinite(B2) || ~isfinite(Ku2M)
        y_sim = [];
        ok = false;
        msg = 'non-finite model parameters.';
        return;
    end

    % Reject unrealistically fast dynamics that usually break simulation.
    nat_freq_proxy = sqrt(k / min(JL, JD));
    if ~isfinite(nat_freq_proxy) || nat_freq_proxy > 2e3
        y_sim = [];
        ok = false;
        msg = 'natural-frequency proxy out of allowed range.';
        return;
    end

    N = length(t_data);
    if length(M_in) ~= N || N < 2
        y_sim = [];
        ok = false;
        msg = 'input/time vector length mismatch or too short.';
        return;
    end

    if any(diff(t_data) <= 0) || any(~isfinite(diff(t_data)))
        y_sim = [];
        ok = false;
        msg = 'time vector is not strictly increasing and finite.';
        return;
    end
    try
        hasSs = exist('ss', 'file') == 2;
        hasC2d = exist('c2d', 'file') == 2;
        hasLsim = exist('lsim', 'file') == 2;

        if ~(hasSs && hasC2d && hasLsim)
            y_sim = [];
            ok = false;
            msg = 'Control System Toolbox not available (ss/c2d/lsim missing).';
            return;
        end

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

        if any(~isfinite(A(:)))
            y_sim = [];
            ok = false;
            msg = 'invalid state-space A matrix.';
            return;
        end

        % Discrete model requested by assignment setup.
        Ts_model = 0.01;
        Gd = c2d(ss(A, B, C, D), Ts_model, 'zoh');

        % Keep time grid aligned to discrete sample time.
        if any(abs(diff(t_data) - Ts_model) > 1e-9)
            y_sim = [];
            ok = false;
            msg = 'time vector must use Ts=0.01 s for the discrete model.';
            return;
        end

        y_sim = lsim(Gd, M_in, t_data);

        if any(~isfinite(y_sim(:)))
            y_sim = [];
            ok = false;
            msg = 'simulation produced NaN/Inf.';
            return;
        end
    catch ME
        y_sim = [];
        ok = false;
        msg = ME.message;
        return;
    end

    ok = true;
    msg = 'ok';
end

% -------------------------------------------------------------------------
function ise = flexDriveFitness(x, t_data, u_pwm, y_meas, lb, ub, x_nom, regWeight, penaltyBase)
    [y_sim, ok] = flexDriveSimulate(x, t_data, u_pwm);

    if ~ok
        span = ub - lb;
        x_norm = (x - lb) ./ span;
        ise = penaltyBase + 1e6 * sum(x_norm.^2);
        return;
    end

    if size(y_sim,1) ~= size(y_meas,1) || size(y_sim,2) ~= size(y_meas,2)
        ise = penaltyBase + 2e6;
        return;
    end

    % Use channel scaling so rpm traces do not dominate angle traces.
    scale = max(std(y_meas, 0, 1), 1e-3);
    err = (y_sim - y_meas) ./ scale;
    dt  = t_data(2) - t_data(1);
    data_cost = dt * sum(sum(err.^2));

    span = ub - lb;
    reg_cost = sum(((x - x_nom) ./ span).^2);
    ise = data_cost + regWeight * reg_cost;

    if ~isfinite(ise)
        ise = penaltyBase + 3e6;
    end
end
