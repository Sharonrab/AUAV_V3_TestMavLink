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
navAzmIdx       = 39;
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
diaFl1Idx       = 56;
diaFl2Idx       = 57;
diaFl3Idx       = 58;
diaSh1Idx       = 59;
diaSh2Idx       = 60;
diaSh3Idx       = 61;
logFl1Idx       = 62;
logFl2Idx       = 63;
logFl3Idx       = 64;
logFl4Idx       = 65;
logFl5Idx       = 66;
logFl6Idx       = 67;

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
M = data;

% Produce the Time vector
time = (M(:, timeStampIdx) - M(1,timeStampIdx))*0.01;



fig_ct = 1;


%% Lat Lon Alt frequency update
figure(fig_ct);

subplot(3,3,1)
 plot(time, M(:, gpsLatIdx), 'r.');
 ylabel('Latitude (deg)');
 
subplot(3,3,2)
 plot(time, M(:, gpsLonIdx), 'r.');
 ylabel('Longitude (deg)');

subplot(3,3,3)
 plot(time, M(:, gpsHeiIdx), 'r.');
 ylabel('Height (m)');
 
subplot(3,3,4)
temp = diff (M(:, gpsLatIdx));
plot (time(1:size(temp,1)), abs(temp));
ylabel('New Latitude');
 
subplot(3,3,5)
temp = diff (M(:, gpsLonIdx));
plot (time(1:size(temp,1)), abs(temp));
ylabel('New Longitude');

subplot(3,3,6)
temp = diff (M(:, gpsHeiIdx));
plot (time(1:size(temp,1)), abs(temp));
ylabel('New Height');

subplot(3,3,7)
plot (time, M(:, gpsFixIdx));
ylabel('GPS Fix');
xlabel('Time (s)');
 
subplot(3,3,8)
plot (time,  M(:, gpsHdoIdx));
ylabel('GPS DOP');
xlabel('Time (s)');

subplot(3,3,9)
plot (time,  M(:, gpsSatIdx));
ylabel('GPS Sats');
xlabel('Time (s)'); 


eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;


%% Lat Lon Plots

figure (fig_ct)

plot(M(:, gpsLonIdx), M(:,gpsLatIdx));
hold
plot(M(:, gpsLonIdx), M(:,gpsLatIdx), 'r.');
hold 
ylabel('Latitude (deg)');
xlabel('Longitude (deg)');
axis equal

eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1

%% X Y Z Plots
figure(fig_ct);

subplot(3,1,1)
 plot(time, M(:, posXIdx), 'r.');
 ylabel('X (m)');
 
subplot(3,1,2)
 plot(time, M(:, posYIdx), 'r.');
 ylabel('Y (m)');

subplot(3,1,3)
 plot(time, M(:, posZIdx), 'r.');
 ylabel('Z (m)');
 xlabel ('Time (s)');
 
eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;

%% XY Plots

figure (fig_ct)

plot(M(:, posYIdx), M(:,posXIdx));
hold
plot(M(:, posYIdx), M(:,posXIdx), 'r.');
hold 
ylabel('X (m)');
xlabel('Y (m)');
axis equal

eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1

%% Roll, Pitch and Yaw comparison
figure(fig_ct);

subplot(3,1,1)
 plot(time, M(:, attRollIdx)*180/pi, 'r');
 ylabel('Roll (deg)');
 
subplot(3,1,2)
 plot(time, M(:, attPitchIdx)*180/pi, 'r');
 ylabel('Pitch (deg)');

subplot(3,1,3)
 plot(time, M(:, attYawIdx)*180/pi, 'r');
 ylabel('Yaw (deg)');
 xlabel ('Time (s)');
 
eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;


%% Commands comparison
figure(fig_ct);

subplot(4,1,1)
 plot(time, M(:, attRollIdx)*180/pi, 'r');
 hold
 plot(time, M(:,navPhicIdx)*180/pi,'b'); 
 hold
 ylabel('Roll vs Roll_c (deg)');
 
subplot(4,1,2)
 plot(time, M(:, attPitchIdx)*180/pi, 'r');
 hold
 plot(time, M(:,navThecIdx)*180/pi,'b'); 
 hold
 ylabel('Pitch vs Pitch_c(deg)');

subplot(4,1,3)
 plot(time, M(:, navUmIdx), 'r');
 hold
 plot (time, ones(size(time))*20);
 hold
 ylabel('U vs U_c (m/s)');
 
subplot(4,1,4)
 plot(time, M(:, navAzmIdx), 'r');
 hold
 plot (time, zeros(size(time)));
 hold
 ylabel('A_z vs A_z_c (m/s)');
 xlabel ('Time (s)');

eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;


%% Control Surfaces
figure(fig_ct);

subplot(5,1,1)
 plot(time, M(:, pwmDtIdx), 'r');
 ylabel('d_T (PWM)');
 
subplot(5,1,2)
 plot(time, M(:, pwmDaIdx), 'r');
 ylabel('d_A(PWM)');

subplot(5,1,3)
 plot(time, M(:, pwmDrIdx), 'r');
 ylabel('D_r(PWM)');
 
subplot(5,1,4)
 plot(time, M(:, pwmDeIdx), 'r');
 ylabel('d_E (PWM)');

 subplot(5,1,5)
 plot(time, M(:, sysModeIdx), 'r');
 ylabel('Nav Mode');
 xlabel ('Time (s)');

eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;

%% Scaled Data

figure(fig_ct)

subplot(4,3,1)
 plot(time, M(:, scaAxIdx), 'r');
 ylabel('Accels (m/s)');

subplot(4,3,2)
 plot(time, M(:, scaAyIdx), 'r');

subplot(4,3,3)
 plot(time, M(:, scaAzIdx), 'r');

subplot(4,3,4)
 plot(time, M(:, scaGxIdx)*180/pi, 'r');
 ylabel('Gyros (deg/s)');

subplot(4,3,5)
 plot(time, M(:, scaGyIdx)*180/pi, 'r');

subplot(4,3,6)
 plot(time, M(:, scaGzIdx)*180/pi, 'r');
 
subplot(4,3,7)
 plot(time, M(:, scaMxIdx), 'r');
 ylabel('Mags (mG)');

subplot(4,3,8)
 plot(time, M(:, scaMyIdx), 'r');

subplot(4,3,9)
 plot(time, M(:, scaMzIdx), 'r');
 
subplot(4,3,10)
 plot(time, M(:, airDynIdx), 'r');
 ylabel('Dynamic Pressure (Pa)');

subplot(4,3,11)
 plot(time, M(:, airStaIdx), 'r');
 ylabel('Static Pressure (Pa)');
 
subplot(4,3,12)
 plot(time, M(:, airTemIdx), 'r');
 ylabel('Air Temperature (C)');
eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;

%% Raw Data

figure(fig_ct)

subplot(4,3,1)
 plot(time, M(:, rawAxIdx), 'r');
 ylabel('Accels (counts)');

subplot(4,3,2)
 plot(time, M(:, rawAyIdx), 'r');

subplot(4,3,3)
 plot(time, M(:, rawAzIdx), 'r');

subplot(4,3,4)
 plot(time, M(:, rawGxIdx), 'r');
 ylabel('Gyros (counts)');

subplot(4,3,5)
 plot(time, M(:, rawGyIdx), 'r');

subplot(4,3,6)
 plot(time, M(:, rawGzIdx), 'r');
 
subplot(4,3,7)
 plot(time, M(:, rawMxIdx), 'r');
 ylabel('Mags (counts)');

subplot(4,3,8)
 plot(time, M(:, rawMyIdx), 'r');

subplot(4,3,9)
 plot(time, M(:, rawMzIdx), 'r');
 
subplot(4,3,10)
 plot(time, M(:, rawPitIdx), 'r');
 ylabel('Dynamic Pressure (counts)');

subplot(4,3,11)
 plot(time, M(:, rawBarIdx), 'r');
 ylabel('Static Pressure (counts)');
 
subplot(4,3,12)
 plot(time, M(:, rawTheIdx), 'r');
 ylabel('Air Temperature (counts)');
eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;

%% Diagnosis
figure(fig_ct)

subplot(4,1,1)
 plot(time, M(:, sysModeIdx), 'b');
 ylabel('Nav Mode');
 hold on;
 plot (time, M(:,sysNavIdx), 'r');
 hold off
 axis tight

subplot(4,1,2)
 plot(time, M(:, navThecIdx), 'b');
 ylabel('\theta_c');
 hold on
 plot(time, M(:, attPitchIdx), 'r');
 hold off
 axis tight
 
subplot(4,1,3)
 plot(time, M(:, navPhicIdx), 'b');
 ylabel('\phi');
 hold on
 plot (time, M(:, attRollIdx), 'r');
 hold off
 axis tight

subplot(4,1,4)
 plot(time, M(:, navPsiDIdx), 'b');
 ylabel('\dot \psi');
 axis tight
 
eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;

%% Voltage/Current Sensor
figure(fig_ct);

subplot(3,1,1)
 plot(time, M(:, pwmDtIdx), 'r');
 axis tight
 ylabel('Throttle');
 
subplot(3,1,2)
 plot(time, M(:, visVolIdx), 'r');
 axis tight
 ylabel('Battery Voltage (Volts)');

subplot(3,1,3)
 plot(time, M(:, visReaIdx), 'r');
 axis tight
 ylabel('Current Consumption(Amps)');
 
 eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;
%% Camera
figure(fig_ct);

subplot(3,1,1)
 plot(time, M(:, ptzZoomIdx), 'r');
 axis tight
 ylabel('Zoom');
 
subplot(3,1,2)
 plot(time, M(:, ptzPanIdx), 'r');
 axis tight
 ylabel('Pan (Degrees)');

subplot(3,1,3)
 plot(time, M(:, ptzTiltIdx), 'r');
 axis tight
 ylabel('Tilt (Degrees)');
 
 eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;
%% 
figure(fig_ct);

subplot(3,1,1)
 plot(time, M(:, rawAxIdx), 'r.');
 ylabel('Accels (counts)');

subplot(3,1,2)
 plot(time, M(:, rawAyIdx), 'r.');

subplot(3,1,3)
 plot(time, M(:, rawAzIdx), 'r.');
 eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;
%% Attic

% %% Start the Main loop
% figure(fig_ct);
% clf;
% 
% x = M(:, posXIdx);
% y = M(:, posYIdx);
% pauseOn =1;
% 
% 
% i = length(time);
% title('Position and L2 Vector');
% 
% axis equal;
% idx = 1:25:i;
% hold on
% 
% for j=1:10:i-1
%     plot(y(j),x(j) ,'-s','MarkerSize',3);
% %     if (mod(j-1,20) == 0)
% %         plot(BP(j,2), BP(j,1), 'rs','MarkerSize',3);
% %         if (AimPt(j,3) ~= -999)
% %             plot(AimPt(j,2), AimPt(j,1),'gs','MarkerSize',3);
% %         end
% %     end
%      %plot the velocity vector
% %      plot ([y(j) y(j)+ve(j)], [x(j) x(j)+vn(j)], 'r');
%      
%      %plot the L2 vector
%      if j > 10 % REN 05/24/10 && L2Enabled(j) == 1
%         plot ([y(j) y(j)+L1(j,2)], [x(j) x(j)+L1(j,1)], 'm-');
%      end
% %      plot N exagerated (multiplied by 20)
% %       plot ([y(j) y(j)+20*N(j,2)], [x(j) x(j)+20*N(j,1)], 'c-');
%       
% % %       %pause the animation
%      if pauseOn == 1
%         if (mod(j-1,10)==0)
%              pause(0.3);
%         end
%      end
%  end
%  plot(y,x ,'-','LineWidth',2.5);
%  xlabel('Y(m)');
%  ylabel('X(m)');
%  grid on;
%  hold off
%  
% %  eval(['print -depsc  '  num2str(figct) '_'  datestr(now,1) '_' ... 
% %      datestr(now,'HH') '_' datestr(now,'MM') '_' datestr(now,'SS')]);
% 
% % +++++++++++++++++++++++++++++++++++++++++
%  figct = figct + 1;
%  %%  Collect the rest of the values
% % %time 
% 
