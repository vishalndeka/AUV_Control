%% 4DOF AUV MPC Model Setup (Surge, Pitch, Yaw, Underactuated Sway)
% physical parameters
m = vehicle.mass;
Iy = vehicle.inertiaMatrix(2, 2); % moment of inertia in pitch
Iz = vehicle.inertiaMatrix(3, 3); % moment of inertia in yaw
Ts = 0.1;     % sample time [s]

% State vector: [x; y; u; v; theta; q; psi; r]
% Input vector: [Fx; My; Mz]

% Continuous-time A matrix (8 states)
A = [ 0  0  1  0  0  0  0  0;
      0  0  0  1  0  0  0  0;
      0  0  0  0  0  0  0  0;
      0  0  0  0  0  0  0  0;
      0  0  0  0  0  1  0  0;
      0  0  0  0  0  0  0  0;
      0  0  0  0  0  0  0  1;
      0  0  0  0  0  0  0  0 ];

% Continuous-time B matrix (3 control inputs)
B = [ 0    0     0;
      0    0     0;
     1/m   0     0;
      0    0     0;
      0    0     0;
      0  1/Iy    0;
      0    0     0;
      0    0   1/Iz ];

C = eye(8);   % Full-state output - all states are observed
D = zeros(8,3);

% Continuous-time state-space system
sys = ss(A, B, C, D);

% Discretize the system
sys_d = c2d(sys, Ts);

% Create MPC controller
mpcobj = mpc(sys_d, Ts);

% Set weights: high priority on x, y, theta, psi tracking
mpcobj.Weights.OutputVariables = [10 10 1 0.1 5 0.5 5 0.5];
mpcobj.Weights.ManipulatedVariables = [0.1 0.05 0.05];
mpcobj.Weights.ManipulatedVariablesRate = [0.2 0.1 0.1];

% Set actuator constraints
mpcobj.MV(1).Min = -5;  mpcobj.MV(1).Max = 5;   % Fx
mpcobj.MV(2).Min = -2.5;   mpcobj.MV(2).Max = 2.5;    % My
mpcobj.MV(3).Min = -0.650;   mpcobj.MV(3).Max = 0.65;    % Mz


% telling MPC that all states are directly measured
mpcobj.Model.Nominal = struct('X', zeros(8,1), 'U', zeros(3,1), 'Y', zeros(8,1));
% setmpcsignals(mpcobj, 'MV', [1 2 3], 'MO', 1:8, 'MD', []);
setEstimator(mpcobj, 'custom');  % disables Kalman filter
mpcobj.Model.Disturbance = ss([], [], [], []); % No disturbance model
mpcobj.Model.Noise = ss([], [], [], []);      % No measurement noise model

% Disable state estimation
setEstimator(mpcobj, 'custom');



% Export to workspace and Simulink
assignin('base', 'mpcobj', mpcobj);
disp('MPC controller "mpcobj" created and exported to base workspace.');
