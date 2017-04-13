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

timeStampIdx =1;%timestamp
%% Prepare the Vectors for plotting
% It assumes the telemtery is stored in a workspace variable named M
% Attitude
% ========

time_boot_ms    = 2;%ATTITUDE.time_boot_ms
attRollIdx      = 3;%ATTITUDE.roll
attPitchIdx     = 4;%ATTITUDE.pitch
attYawIdx       = 5;%ATTITUDE.yaw
attPIdx         = 6;%ATTITUDE.rollspeed
attQIdx         = 7;%ATTITUDE.pitchspeed
attRIdx         = 8;%ATTITUDE.yawspeed

% Position
% ========

LOCAL_POSITION_NED_time_boot_ms = 9;%LOCAL_POSITION_NED.time_boot_ms
posXIdx         = 10;%LOCAL_POSITION_NED.x
posYIdx         = 11;%LOCAL_POSITION_NED.y
posZIdx         = 12;%LOCAL_POSITION_NED.z
posVxIdx        = 13;%LOCAL_POSITION_NED.vx
posVyIdx        = 14;%LOCAL_POSITION_NED.vy
posVzIdx        = 15;%LOCAL_POSITION_NED.vz


% GPS
% ===

GPS_RAW_INT_time_usec = 16;%GPS_RAW_INT.time_usec
gpsFixIdx       = 17;%GPS_RAW_INT.fix_type
gpsLatIdx       = 18;%GPS_RAW_INT.lat
gpsLonIdx       = 19;%GPS_RAW_INT.lon
gpsHeiIdx       = 20;%GPS_RAW_INT.alt
gpsEphIdx       = 21;%GPS_RAW_INT.eph
gpsEpvIdx       = 22; %GPS_RAW_INT.epv
gpsSogIdx       = 23;%GPS_RAW_INT.vel
gpsCogIdx       = 24;%GPS_RAW_INT.cog
gpsHdoIdx       = 25;%GPS_RAW_INT.satellites_visible
gpsYrIdx        = 26;%GPS_DATE_TIME.year
gpsMoIdx        = 27;%GPS_DATE_TIME.month
gpsDyIdx        = 28;%GPS_DATE_TIME.day
gpsHrIdx        = 29;%GPS_DATE_TIME.hour
gpsMnIdx        = 30;%GPS_DATE_TIME.min
gpsScIdx        = 31;%GPS_DATE_TIME.sec
gpsCkStatIdx    = 32;%GPS_DATE_TIME.clockStat
gpsSatIdx       = 33;%GPS_DATE_TIME.visSat 
gpsUseSatIdx    = 34;%GPS_DATE_TIME.useSat
gpsGpGlIdx      = 35;%GPS_DATE_TIME.GppGl
gpsMaskdx       = 36;%GPS_DATE_TIME.sigUsedMask
gpsPercentIdx   = 37;%GPS_DATE_TIME.percentUsed


% Navigation
% ==========

navUmIdx        = 38;%SLUGS_NAVIGATION.u_m
navPhicIdx      = 39;%SLUGS_NAVIGATION.phi_c
navThecIdx      = 40;%SLUGS_NAVIGATION.theta_c
navPsiDIdx      = 41;%SLUGS_NAVIGATION.psiDot_c
navAzmIdx       = 42;%SLUGS_NAVIGATION.ay_body
navDis2GoIdx    = 43;%SLUGS_NAVIGATION.totalDist
navRemIdx       = 44;%SLUGS_NAVIGATION.dist2Go
navWp1Idx       = 45;%SLUGS_NAVIGATION.fromWP
navWp2Idx       = 46;%SLUGS_NAVIGATION.toWP
navHcIdx        = 47;%SLUGS_NAVIGATION.h_c
% Raw Data
% ========

RAW_IMU_time_usec=48;%RAW_IMU.time_usec
rawAxIdx        = 49 ;%RAW_IMU.xacc
rawAyIdx        = 50 ;%RAW_IMU.yacc
rawAzIdx        = 51 ;%RAW_IMU.zacc
rawGxIdx        = 52;%RAW_IMU.xgyro
rawGyIdx        = 53 ;%RAW_IMU.ygyro
rawGzIdx        = 54 ;%RAW_IMU.zgyro
rawMxIdx        = 55 ;%RAW_IMU.xmag
rawMyIdx        = 56 ;%RAW_IMU.ymag
rawMzIdx        = 57 ;%RAW_IMU.zmag

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

SCALED_PRESSURE_time_boot_ms = 58;%SCALED_PRESSURE.time_boot_ms
airDynIdx       = 59;%SCALED_PRESSURE.press_abs
airStaIdx       = 60;%SCALED_PRESSURE.press_diff
airTemIdx       = 61;%SCALED_PRESSURE.temperature

% Diagnostic
% ==========
% diaFl1Idx       = 56; %Turn_Lead_D {56} 
% diaFl2Idx       = 57; % IP_Reach {57} 
% diaFl3Idx       = 58; % L2 {58} 
% diaSh1Idx       = 59; % RTB {59} 
% diaSh2Idx       = 60; % WP Index {60} 
% diaSh3Idx       = 61; % WP Fly {61} 
% logFl1Idx       = 62; % WPI_X {62} 
% logFl2Idx       = 63; % WPI_Y {63} 
% logFl3Idx       = 64; % WPI_Z {64} 
% logFl4Idx       = 65; % L2_X {65} 
% logFl5Idx       = 66; % L2_Y {66}
% logFl6Idx       = 67; % L2_Z {67}

% CPU
% ===

cpuSensIdx      = 62;%CPU_LOAD.sensLoad
cpuCtrlIdx      = 63;%CPU_LOAD.ctrlLoad
cpuBatIdx       = 64;%CPU_LOAD.batVolt
% HEARTBEAT
% =======

%HEARTBEAT.autopilot,HEARTBEAT.base_mode,HEARTBEAT.custom_mode,HEARTBEAT.system_status,HEARTBEAT.mavlink_version,
% 5 indexes free
% System
% ======


sysModeIdx      = 71;%SYS_STATUS.onboard_control_sensors_present
sysNavIdx       = 72;%SYS_STATUS.onboard_control_sensors_enabled
sysStatIdx      = 73;%SYS_STATUS.onboard_control_sensors_health
sysLoadIdx      = 74;%SYS_STATUS.load
sysBatVIdx      = 75;%SYS_STATUS.voltage_battery
sysBatRIdx      = 76;%SYS_STATUS.current_battery
sysPacDIdx      = 77;%SYS_STATUS.battery_remaining
%SYS_STATUS.drop_rate_comm,SYS_STATUS.errors_comm,SYS_STATUS.errors_count1,SYS_STATUS.errors_count2,SYS_STATUS.errors_count3,SYS_STATUS.errors_count4
% 6 indexes free to 83
% Servos 
% ======

RC_CHANNELS_RAW_time_boot_ms = 84;%RC_CHANNELS_RAW.time_boot_ms
%RC_CHANNELS_RAW.port 85
pilDtIdx        = 86;%RC_CHANNELS_RAW.chan1_raw
pilDaIdx        = 87;%RC_CHANNELS_RAW.chan2_raw
pilDrIdx        = 88;%RC_CHANNELS_RAW.chan3_raw
pilDeIdx        = 89;%RC_CHANNELS_RAW.chan4_raw

pwmDtIdx        = 90;%RC_CHANNELS_RAW.chan5_raw
pwmDaIdx        = 91;%RC_CHANNELS_RAW.chan6_raw
pwmDrIdx        = 92;%RC_CHANNELS_RAW.chan7_raw
pwmDeIdx        = 93;%RC_CHANNELS_RAW.chan8_raw
%RC_CHANNELS_RAW.rssi 94
%SERVO_OUTPUT_RAW.time_usec 95
%SERVO_OUTPUT_RAW.port 96
pilDtTIdx       = 97;%SERVO_OUTPUT_RAW.servo1_raw
pilDaTIdx       = 98;%SERVO_OUTPUT_RAW.servo2_raw
pilDrTIdx       = 99;%SERVO_OUTPUT_RAW.servo3_raw
pilDeTIdx       = 100;%SERVO_OUTPUT_RAW.servo4_raw
%SERVO_OUTPUT_RAW.servo5_raw,SERVO_OUTPUT_RAW.servo6_raw,SERVO_OUTPUT_RAW.servo7_raw,SERVO_OUTPUT_RAW.servo8_raw
% to 103
% Scaled Data
% ===========
%SCALED_IMU.time_boot_ms,SCALED_IMU.xacc,SCALED_IMU.yacc,SCALED_IMU.zacc,SCALED_IMU.xgyro,SCALED_IMU.ygyro,SCALED_IMU.zgyro,SCALED_IMU.xmag,SCALED_IMU.ymag,SCALED_IMU.zmag,RAW_PRESSURE.time_usec,RAW_PRESSURE.press_abs,RAW_PRESSURE.press_diff1,RAW_PRESSURE.press_diff2,RAW_PRESSURE.temperature,MID_LVL_CMDS.target,MID_LVL_CMDS.hCommand,MID_LVL_CMDS.uCommand,MID_LVL_CMDS.rCommand,SENSOR_DIAG.float1,SENSOR_DIAG.float2,SENSOR_DIAG.int1,SENSOR_DIAG.char1,VOLT_SENSOR.r2Type,VOLT_SENSOR.voltage,VOLT_SENSOR.reading2,STATUS_GPS.csFails,STATUS_GPS.gpsQuality,STATUS_GPS.msgsType,STATUS_GPS.posStatus,STATUS_GPS.magVar,STATUS_GPS.magDir,STATUS_GPS.modeInd,NOVATEL_DIAG.timeStatus,NOVATEL_DIAG.receiverStatus,NOVATEL_DIAG.solStatus,NOVATEL_DIAG.posType,NOVATEL_DIAG.velType,NOVATEL_DIAG.posSolAge,NOVATEL_DIAG.csFails,PTZ_STATUS.zoom,PTZ_STATUS.pan,PTZ_STATUS.tilt
% scaAxIdx        = 90;
% scaAyIdx        = 91;
% scaAzIdx        = 92;
% scaGxIdx        = 93;
% scaGyIdx        = 94;
% scaGzIdx        = 95;
% scaMxIdx        = 96;
% scaMyIdx        = 97;
% scaMzIdx        = 98;

% Raw Pressures
% =============
% rawBarIdx       = 99 ;
% rawPitIdx       = 100 ;
% rawPwrIdx       = 101 ;
% rawTheIdx       = 102 ;
% 
% navHcIdx        = 103;

% Sensor Diagnostics
%===================
% senFl1Idx       =104;
% senFl2Idx       =105;
% senIn1Idx       =106;
% senCh1Idx       =107;

%VI Sensor
% ========
% visTypIdx       =108;
% visVolIdx       =109;
% visReaIdx       =110;

% GPS Status
% ==========

% Novatel Status
% ==============
% novTsIdx        =118;
% novRsIdx        =119;
% novSsIdx        =120;
% novPtIdx        =121;
% novVtIdx        =122;
% novPaIdx        =123;
% novFaIdx        =124;

% Pan Tilt
% ========
% ptzZoomIdx      =125;
% ptzPanIdx       =126;
% ptzTiltIdx      =127;
% 
% pilFailIdx      = 128;

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


