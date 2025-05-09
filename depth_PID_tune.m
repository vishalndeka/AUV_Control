% Depth-hold PD tuning for AUV
% clear; clc;

% AUV Parameters
m = 123.3861;         % mass [kg]
D = 100;              % vertical damping [Ns/m] — adjust to match your model
z_ref = 3;            % target depth [m]
sim_time = 30;        % simulation time [s]
dt = 0.01;            % time step

% Tuning ranges
Kp_vals = linspace(10, 300, 10);
Kd_vals = linspace(0.1, 50, 10);

% Logging
best_score = inf;
best_Kp = 0;
best_Kd = 0;

for Kp = Kp_vals
    for Kd = Kd_vals
        % Initial states
        z = 0; w = 0;
        z_log = zeros(sim_time/dt,1);
        
        for t = 1:length(z_log)
            err = z_ref - z;

            % PD controller
            Fz = Kp * err - Kd * w;
            Fz = max(min(Fz, 40), -40);  % limit actuator force

            % AUV heave dynamics
            dw = (Fz - D * w) / m;
            w = w + dw * dt;
            z = z + w * dt;

            z_log(t) = z;
        end

        % Evaluate performance (IAE)
        error = abs(z_log - z_ref);
        score = sum(error) * dt;

        if score < best_score
            best_score = score;
            best_Kp = Kp;
            best_Kd = Kd;
        end
    end
end

fprintf('Best Kp = %.2f, Kd = %.2f (IAE = %.3f)\n', best_Kp, best_Kd, best_score);
