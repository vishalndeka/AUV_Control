%% Vehicle Parameters Setup

Environment.rho = 1000; % Freshwater density
Environment.g = 9.81; % Earth's gravitational acceleration
vehicle.totalMass = 123.3861;
vehicle.body.radius = 0.138719;
vehicle.body.length = 0.5127105;%-(2*vehicle.body.radius); % metres

vehicle.inertiaMatrix = [1.748560020850131,-5.150339498968297e-04,0.161156618079729;
                        -5.150339498968297e-04,32.006474703109390,6.847426795052851e-04;
                        0.161156618079729,6.847426795052851e-04,32.198164916583416];
vehicle.buoyancyForce = 1210.417641;

% added mass matrix from Mathwork's AUV
vehicle.added_mass = [2.86157179447936	0	0	0	0	0;
0	125.094277772980	0	0	0	-16.6286723830141;
0	0	125.094277772980	0	-16.6286723830141	0;
0	0	0	0	0	0;
0	0	-16.6286723830141	0	37.9086951431153	0;
0	-16.6286723830141	0	0	0	37.9086951431153];
