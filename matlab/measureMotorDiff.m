n = 32;
pwm_values = zeros(1,n);
gains = zeros(1,n);

for k = 1:n
    pwm = k * 125;

    fprintf('Iteration: %d\n', k);
    fprintf('PWM: %g\n', pwm);

    simOut = sim("model.slx", "StopTime", "20");

    
    pause
end