n = 16;
pwm_values = zeros(1,n);
gains = zeros(1,n);

for k = 1:n
    pwm = k * 250;

    fprintf('Iteration: %d\n', k);
    fprintf('PWM: %g\n', pwm);

    simOut = sim("model.slx", "StopTime", "600");

    simdata = simOut.velocities;

    v1_average = mean(simdata(30000:end,1))
    v2_average = mean(simdata(30000:end,2))

    gains(k) = v1_average / v2_average;

    fprintf('Gain: %g\n', gains(k));
end