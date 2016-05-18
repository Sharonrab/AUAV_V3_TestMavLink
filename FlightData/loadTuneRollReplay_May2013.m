% load the workspace named replaySample.mat

clear
GetDataIndexes  %load indices

%load 'PitotCal'%'AltitudeAndRoll'%
%load 'AltitudeAndRoll'%
%load 'CrashLog'
load 'PitotCal_0'
% clear the time 
clear time 
apSampleTime = 0.01;

TData = 0.01;
Ts = apSampleTime;
% You need to have a data set called data in the workspace 
% for the model to run. It is used to compute the time vector you can also
% get a "subselection" of data, see the next comment section.


% generate the time vector to match the reloaded data set.
time = (data(:, timeStampIdx) - data(1,timeStampIdx))*0.01;


% filter initialization
% U_comm = 20; 
run ..\apConfiguration\Environment.m 
% run ..\apConfiguration\limits.m 
run ..\apConfiguration\compFilterInit.m 
% run ..\apConfiguration\NavData.m %Uses U_comm so must follow NavData.m
run ..\apConfiguration\baroInit.m
run ..\apConfiguration\sensorInit.m 

% GS Location. East field UCSC location
GS_location = [180 36.9898376464844 -122.051963806152];
baseHeight = GS_location(1);

% using mag field for 95064 
% MagIntensity = [ 23035	5636 42151 ];
% UnitMag_NED = MagIntensity/norm(MagIntensity);
% UnitMag_NE = [MagIntensity(1) MagIntensity(2) 0];
% UnitMag_NE = UnitMag_NE/norm(UnitMag_NE);
% compFilterInit
