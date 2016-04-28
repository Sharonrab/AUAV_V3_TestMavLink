%% General Values
%path(path,'C:\Documents and Settings\Administrator\My Documents\apcode');

derivativesConstant = 5;

apSampleTime = 0.01;
T = 0.01;
SampleT=0.005;


%% Run the Required Configuration Files for Simulation
run ..\apConfiguration\Rascal_Var.m  %UAV model parameters
run ..\apConfiguration\compFilterInit.m

% ===== Replace this one with your location file ====
run ..\apConfiguration\gsLocation.m
% ===================================================

run ..\apConfiguration\simulationWPFile.m  %waypoint location
run ..\apConfiguration\failuresInit.m
run ..\apConfiguration\Environment.m  %atmosphere and wind
run ..\apConfiguration\pwmConversions.m
run ..\apConfiguration\limits.m
run ..\apConfiguration\NavData.m      %vehicle initial conditions and commanded speed
                                      %must follow limits.m and NavData.m

run ..\apConfiguration\baroInit.m
run ..\apConfiguration\L2Plus_IP_RTB.m  %constants for tracking and homing logic

%% Random Initialization
%run .\batchTests\randomInit.m

