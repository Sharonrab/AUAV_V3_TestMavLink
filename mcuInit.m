%% General values
% Global Sample Time
T = 0.01;
apSampleTime = 0.01;

derivativesConstant = 5;
%PAR_HIL =1;
%%%%%
% the vars below are not used in the code they belong to InnereOuter sim
% BUT you need to have them in order for the code generator to work
Xpoints = [0 0]; %DO NOT DELETE - Has no effect on code 
Ypoints = [0 0]; %DO NOT DELETE - Has no effect on code
Zpoints = [0 0]; %DO NOT DELETE - Has no effect on code
latlong_WP = 1;  %DO NOT DELETE - Has no effect on code
%%%%%

%% Run the configuration files
run .\apConfiguration\Rascal_Var.m

% ===== Replace this one with your location file ====
 run .\apConfiguration\gsLocation.m
% ===================================================

run .\apConfiguration\Environment.m
%run .\apConfiguration\pwmConversionsVANT01.m
%run .\apConfiguration\AM_Mentor_PwmConversions.m
run .\apConfiguration\AUAV3_PwmConversions.m
run .\apConfiguration\limits.m
run .\apConfiguration\baroInit.m
run .\apConfiguration\sensorInit.m
run .\apConfiguration\compFilterInit.m
run .\apConfiguration\parameterEnums.m
run .\apConfiguration\L2Plus_IP_RTB.m
run .\apConfiguration\panTiltInitVANT01.m

