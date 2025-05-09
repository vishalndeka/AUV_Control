%% Starting Afresh

% Vehicle properties;
vehicle.body.htHolderMass = 0.405;
vehicle.body.lowerClampMass = 0.152;
vehicle.body.camDomeMass = 0.140;
vehicle.body.camEcMass = 0.132;
vehicle.body.vtHolderFrontMass = 0.259;
vehicle.body.vtHolderRearMass = 0.380;
vehicle.body.connectorEcMass = 0.150;
vehicle.body.basePlateMass = 0.168;

vehicle.body.jetsonMass = 0.248;
vehicle.body.batteryMass = 0.700;
vehicle.body.motorMass = 0.172;
vehicle.body.px4Mass = 0.038;
vehicle.body.escMass = 0.024;
vehicle.body.pdbMass = 0.050;

vehicle.totalMass = vehicle.body.htHolderMass+vehicle.body.lowerClampMass+vehicle.body.camDomeMass+...
    vehicle.body.camEcMass+vehicle.body.vtHolderFrontMass+vehicle.body.vtHolderRearMass+...
    (2*vehicle.body.connectorEcMass)+vehicle.body.basePlateMass+vehicle.body.jetsonMass+...
    (3*vehicle.body.batteryMass)+(4*vehicle.body.motorMass)+vehicle.body.px4Mass+(4*vehicle.body.escMass)+...
    vehicle.body.pdbMass;

% vehicle.body.totalMass = 8.5;

% vehicle.body.cylinderMass = vehicle.body.htHolderMass+vehicle.body.lowerClampMass+(2*vehicle.body.connectorEcMass)+...
%     vehicle.body.basePlateMass+vehicle.body.jetsonMass+(3*vehicle.body.batteryMass)+(4*vehicle.body.motorMass)+...
%     vehicle.body.px4Mass+(4*vehicle.body.escMass)+vehicle.body.pdbMass;
% 
% vehicle.body.frontHemiMass = 

% Volume of water displaced by the drone when its 5.156 kilos and neutrally buoyant:
% preliminary calculations:

Environment.rho = 1000;
Environment.g = 9.81;

vehicle.body.radius = 0.062;
vehicle.body.length = vehicle.totalMass/(Environment.rho*pi*vehicle.body.radius^2)-(4/3)*vehicle.body.radius; % this length of the cylinder ensures the drone is neutrally buoyant (somewhat)

vehicle.buoyancyForce = Environment.rho*Environment.g*((4*pi*vehicle.body.radius^3/3)+(pi*vehicle.body.radius^2*vehicle.body.length));
vehicle.inertiaMatrix = zeros(3, 3);
mass_hemisphere = 0.2;
mass_cylinder = vehicle.totalMass-(2*mass_hemisphere);
I_x = 0.5*mass_cylinder*vehicle.body.radius^2+(83/320*mass_hemisphere*vehicle.body.radius^2);
I_y = 1/12*mass_cylinder*(3*vehicle.body.radius^2+vehicle.body.length^2)+2*(0.4*mass_hemisphere*vehicle.body.radius^2);
I_z = I_y; % because symmetry

vehicle.inertiaMatrix(1, 1) = I_x;
vehicle.inertiaMatrix(2, 2) = I_y;
vehicle.inertiaMatrix(3, 3) = I_z;

%%
% The Added mass matrix calculation

% Volumes
volume_cylinder = pi * vehicle.body.radius^2 * vehicle.body.length;
volume_hemisphere = (2/3) * pi * vehicle.body.radius^3;
volume_total = volume_cylinder + 2 * volume_hemisphere;
L = vehicle.body.length+2*vehicle.body.radius;

% Added mass coefficients
CX = 0.1;    % Surge
CY = 1.0;    % Sway
CZ = 1.0;    % Heave
CK = 0.1;    % Roll
CM = 0.2;    % Pitch
CN = 0.2;    % Yaw

% Translational added masses
X_du = -CX * Environment.rho * volume_total;
Y_dv = -CY * Environment.rho * volume_total;
Z_dw = -CZ * Environment.rho * volume_total;

% Rotational added masses
K_dp = -CK * Environment.rho * volume_total * vehicle.body.radius^2;
M_dq = -CM * Environment.rho * volume_total * L^2;
N_dr = -CN * Environment.rho * volume_total * L^2;

% Added mass matrix (6x6)
vehicle.added_mass = diag([X_du, Y_dv, Z_dw, K_dp, M_dq, N_dr]);
