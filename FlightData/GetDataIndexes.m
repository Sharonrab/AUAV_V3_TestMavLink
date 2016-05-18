%% read the data
% data = dlmread('Z:\VMShared\ValuesGPSJueves-MAV 100.txt', ',', 150,0);
% 

% enum MAV_MODE
% {
%     MAV_MODE_UNINIT = 0,     ///< System is in undefined state
%     MAV_MODE_LOCKED = 1,     ///< Motors are blocked, system is safe
%     MAV_MODE_MANUAL = 2,     ///< System is allowed to be active, under manual (RC) control
%     MAV_MODE_GUIDED = 3,     ///< System is allowed to be active, under autonomous control, manual setpoint
%     MAV_MODE_AUTO =   4,     ///< System is allowed to be active, under autonomous control and navigation
%     MAV_MODE_TEST1 =  5,     ///< Generic test mode, for custom use
%     MAV_MODE_TEST2 =  6,     ///< Generic test mode, for custom use
%     MAV_MODE_TEST3 =  7,     ///< Generic test mode, for custom use
%     MAV_MODE_READY =  8,     ///< System is ready, motors are unblocked, but controllers are inactive
%     MAV_MODE_RC_TRAINING = 9 ///< System is blocked, only RC valued are read and reported back
% };



% MAVLink's Nav supported modes are:
% 
% enum MAV_NAV
% {
%     MAV_NAV_GROUNDED = 0,
%     MAV_NAV_LIFTOFF,
%     MAV_NAV_PASSTHROUGH,
%     MAV_NAV_WAYPOINT,
%     MAV_NAV_MID_LEVEL,
%     MAV_NAV_RETURNING,
%     MAV_NAV_LANDING,
%     MAV_NAV_LOST,
%     MAV_NAV_SEL_PT,
%     MAV_NAV_ISR,
%     MAV_NAV_LINE_PATROL
% };
% 
% These are only relevant when SLUGS mode is in MAV_MODE_GUIDED.
% SLUGS uses the nav modes as follows:
% 
% MAV_NAV_MID_LEVEL: Autonomous mode but using mid-level commands, the end user must set a command for airspeed, altitude and turnrate
% 
% MAV_NAV_WAYPOINT: Fully autonomous mode using waypoint navigation. The end user configures the waypoints and the commanded airspeed.
% 
% MAV_NAV_PASSTHROUGH: Passthrough mode. The Pilot commands are passed through the autopilot and sent as if it were autopilot generated
% 
% MAV_NAV_SEL_PT: Selective passtrough. Some pilot commands (selectively chosen form the ground station) are passed and others are used from the autopilot
%                                                         as if in GUIDED mode.

% MAV_NAV_ISR: Flying in circles over a coordinate


%% Prepare the Vectors for plotting
% It assumes the telemtery is stored in a workspace variable named M
% Attitude
% ========
attRollIdx      = 1;
attPitchIdx     = 2;
attYawIdx       = 3;
attPIdx         = 4;
attQIdx         = 5;
attRIdx         = 6;
timeStampIdx    = 7;

% Position
% ========
posXIdx         = 8;
posYIdx         = 9;
posZIdx         = 10;
posVxIdx        = 11;
posVyIdx        = 12;
posVzIdx        = 13;


% GPS
% ===
gpsFixIdx       = 15; %% ======================= 14
gpsLatIdx       = 16;
gpsLonIdx       = 17;
gpsHeiIdx       = 18;
gpsSogIdx       = 19;
gpsCogIdx       = 20;
gpsHdoIdx       = 21; %% ====
gpsYrIdx        = 23;
gpsMoIdx        = 24;
gpsDyIdx        = 25;
gpsHrIdx        = 26;
gpsMnIdx        = 27;
gpsScIdx        = 28;
gpsSatIdx       = 29;
gpsCkStatIdx    = 30;
gpsUseSatIdx    = 31;
gpsGpGlIdx      = 32;
gpsMaskdx       = 33;
gpsPercentIdx   = 34;


% Navigation
% ==========
navUmIdx        = 35;
navPhicIdx      = 36;
navThecIdx      = 37;
navPsiDIdx      = 38;
navAzmIdx       = 39;%actually navAymIdx
navDis2GoIdx    = 40;
navRemIdx       = 41;
navWp1Idx       = 42;
navWp2Idx       = 43;

% Raw Data
% ========
rawAxIdx        = 44 ;
rawAyIdx        = 45 ;
rawAzIdx        = 46 ;
rawGxIdx        = 47;
rawGyIdx        = 48 ;
rawGzIdx        = 49 ;
rawMxIdx        = 50 ;
rawMyIdx        = 51 ;
rawMzIdx        = 52 ;

% Bias
% ====
% biaAxIdx        = 48;
% biaAyIdx        = 49;
% biaAzIdx        = 50;
% biaGxIdx        = 51;
% biaGyIdx        = 52;
% biaGzIdx        = 53;

% Air data
% ========
airDynIdx       = 54;
airStaIdx       = 53;
airTemIdx       = 55;

% Diagnostic
% ==========
diaFl1Idx       = 56; %Turn_Lead_D {56} 
diaFl2Idx       = 57; % IP_Reach {57} 
diaFl3Idx       = 58; % L2 {58} 
diaSh1Idx       = 59; % RTB {59} 
diaSh2Idx       = 60; % WP Index {60} 
diaSh3Idx       = 61; % WP Fly {61} 
logFl1Idx       = 62; % WPI_X {62} 
logFl2Idx       = 63; % WPI_Y {63} 
logFl3Idx       = 64; % WPI_Z {64} 
logFl4Idx       = 65; % L2_X {65} 
logFl5Idx       = 66; % L2_Y {66}
logFl6Idx       = 67; % L2_Z {67}

% CPU
% ===
cpuBatIdx       = 68;
cpuCtrlIdx      = 69;
cpuSensIdx      = 70;

% System
% ======
sysModeIdx      = 71;
sysNavIdx       = 72;
sysStatIdx      = 73;
sysLoadIdx      = 74;
sysBatVIdx      = 75;
sysBatRIdx      = 76;
sysPacDIdx      = 77;

% Servos 
% ======
pilDtIdx        = 78;
pilDaIdx        = 79;
pilDrIdx        = 80;
pilDeIdx        = 81;

pwmDtIdx        = 82;
pwmDaIdx        = 83;
pwmDrIdx        = 84;
pwmDeIdx        = 85;

pilDtTIdx       = 86;
pilDaTIdx       = 87;
pilDrTIdx       = 88;
pilDeTIdx       = 89;

% Scaled Data
% ===========
scaAxIdx        = 90;
scaAyIdx        = 91;
scaAzIdx        = 92;
scaGxIdx        = 93;
scaGyIdx        = 94;
scaGzIdx        = 95;
scaMxIdx        = 96;
scaMyIdx        = 97;
scaMzIdx        = 98;

% Raw Pressures
% =============
rawBarIdx       = 99 ;
rawPitIdx       = 100 ;
rawPwrIdx       = 101 ;
rawTheIdx       = 102 ;

navHcIdx        = 103;

% Sensor Diagnostics
%===================
senFl1Idx       =104;
senFl2Idx       =105;
senIn1Idx       =106;
senCh1Idx       =107;

%VI Sensor
% ========
visTypIdx       =108;
visVolIdx       =109;
visReaIdx       =110;

% GPS Status
% ==========

% Novatel Status
% ==============
novTsIdx        =118;
novRsIdx        =119;
novSsIdx        =120;
novPtIdx        =121;
novVtIdx        =122;
novPaIdx        =123;
novFaIdx        =124;

% Pan Tilt
% ========
ptzZoomIdx      =125;
ptzPanIdx       =126;
ptzTiltIdx      =127;

pilFailIdx      = 128;

%% Way to obtain Waypoint Nav Only
%
% To plot everything, simply set M = data;
%
%idx = find ((data (:, sysModeIdx) == 3));

%M = data (idx,:);
% 
% idx = find(data(:, sysNavIdx) == 9);
% M = data(idx,:);

% idx = 6000:1:11000;
% 
% M = data(idx,:);
% M = data;
% 
% % Produce the Time vector
% time = (M(:, timeStampIdx) - M(1,timeStampIdx))*0.01;


