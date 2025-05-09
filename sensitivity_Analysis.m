% Define test cases
surge_speeds = [0.5, 1.0, 1.5];           % Surge speeds (u0) in m/s
yaw_rates = deg2rad([0, 5, 10]);         % Initial yaw rates (r0) in rad/s

% Simulink model name
simModel = 'AUV_Model';  % <-- Replace with your actual model name

% Initialize results structure
results = struct;

for i = 1:length(surge_speeds)
    for j = 1:length(yaw_rates)
        % Create simulation input
        in = Simulink.SimulationInput(simModel);
        
        % Set initial conditions
        in = in.setVariable('u0', surge_speeds(i));  % Initial surge velocity
        in = in.setVariable('r0', yaw_rates(j));     % Initial yaw rate

        % Run the simulation
        simOut = sim(in);

        % Extract time and yaw angle response
        time = simOut.tout;
        yaw_angle = simOut.logsout.getElement('yaw').Values.Data;  % <-- Replace with actual signal name
        actuator_force = simOut.logsout.getElement('Fx').Values.Data;  % Replace with correct actuator signal if needed

        % Analyze yaw angle response
        stepInfo = stepinfo(yaw_angle, time);

        % Store results
        label = sprintf('u=%.1f_r=%.1f', surge_speeds(i), rad2deg(yaw_rates(j)));
        results.(label).RiseTime = stepInfo.RiseTime;
        results.(label).Overshoot = stepInfo.Overshoot;
        results.(label).SettlingTime = stepInfo.SettlingTime;
        results.(label).MaxActuator = max(abs(actuator_force));
    end
end

% Display one result for verification
disp(results.('u=1.0_r=10.0'))
