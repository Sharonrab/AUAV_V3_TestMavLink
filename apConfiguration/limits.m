%% Value Definitions
aileronLimit = 22; % 45 for HIL, 25 for flights
bankLimit = 25;    %45 for HIL, 22 for flight

%% Control Surface Limits
deLimit = single(23*pi/180);
daLimit = single(aileronLimit*pi/180);
dtLimit = single(0.95);
drLimit = single(10*pi/180);

%% Trim Limits IN IC 
% divide by 5 to get uS
dtTrimMin = ((0.55)-bICdt)/(mICdt);%((0.20)-bICdt)/(mICdt); AM 4 new Longi 9386;%
dtTrimMax = ((0.68)-bICdt)/(mICdt);%((0.80)-bICdt)/(mICdt); AM 4 new Longi 14000;%

daTrimMin = ((2*pi/180)-bICda)/(mICda);%((-2*pi/180)-bICda)/(mICda); AM reverse servo 11000;%
daTrimMax = ((-2*pi/180)-bICda)/(mICda);%((2*pi/180)-bICda)/(mICda); AM reverse servo 14000;%

drTrimMin = ((2*pi/180)-bICdr)/(mICdr);%((-2*pi/180)-bICdr)/(mICdr); AM reverse servo 11000;%
drTrimMax = ((-2*pi/180)-bICdr)/(mICdr);%((2*pi/180)-bICdr)/(mICdr); AM reverse servo 14000;%

deTrimMin = ((-2*pi/180)-bICde)/(mICde);%11000;%
deTrimMax = ((2*pi/180)-bICde)/(mICde);%14000;%



%% Command Limits
pitchCommandLimit = single(15*pi/180);
bankCommandLimit = single(bankLimit*pi/180);

%% ?
r_lim=200*pi/180;
 
%% Navigation
maxAcc = single(ISA_g*tan(bankCommandLimit));

U_comm = single(19.5);

% this requires Environment.m amd limits.m to be loaded first
% factor of 1.1 on max ground speed is for contingencies 
% R Min
MinTurnRadius = single((1.1*(U_comm)^2)/maxAcc);
