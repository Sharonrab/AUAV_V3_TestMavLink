%% Set up the values for the HIL Simulation

% Do not change this variable unless you understand what you are doing.
% This sets the sampling ratio for UDP send/receive data.

HIL_K = 3;
SampleT = 0.005;
apSampleTime = 0.01;

%% Run the standard setup files
run ..\apConfiguration\Rascal_Var.m

% ===== Replace this one with your location file ====
run ..\apConfiguration\gsLocation.m
% ===================================================

run ..\apConfiguration\Environment.m
run ..\apConfiguration\failuresInit.m
run ..\apConfiguration\AM_Mentor_PwmConversions.m
run ..\apConfiguration\limits.m
run ..\apConfiguration\baroInit.m
run ..\apConfiguration\sensorInit.m
run ..\apConfiguration\NavData.m

%% Override NavData values
Pos_0   = [0; 0; 150]';     % Initial position vector (m)
Euler_0 = [0; 0*pi/180; 0*pi/180]';     % Initial Euler angles    (rad)
Vb_0    = [ 17; 0; 0]';   % Initial body-velocity vector (m/s)

%% Wind Configuration
windBase = 3.5;
windDirTurb = 180;
windDirHor = 180;