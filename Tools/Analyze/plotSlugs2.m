
clear
GetDataIndexes  %load indices
load 'my_csv_log_file.mat'
%load 'my_csv_log_file'
%time = (data(:, timeStampIdx) - data(1,timeStampIdx))*0.01;
k =find(data(:,timeStampIdx)<6050);
sz = size(k);

if sz(:,1)==0
    time = data(:,timeStampIdx);
else
    data(k,timeStampIdx)= data(k,timeStampIdx)*1000;
    time = data(:,timeStampIdx);
%     time = data(1:k(1)-1,timeStampIdx);
%     for i=1:sz(1)-1
%     time = [time; data(k(i):k(i+1)-1,timeStampIdx)+data(k(i)-1,timeStampIdx)];
%     %time = [data(1:110,1); data(111:376,1)+data(110,1) ;data(377:end,1)+data(376,1)+data(110,1) ];
%     end
%     time = [time; data(k(i+1):end,timeStampIdx)+data(k(i)-1,timeStampIdx)+data(k(i+1)-1,timeStampIdx)];

end
time = (time(:, timeStampIdx) - time(1,timeStampIdx))*0.01;

M = data;

% Produce the Time vector
%time = (M(:, timeStampIdx) - M(1,timeStampIdx))*0.01;



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
% plot(time, M(:, pwmDtIdx), 'r');
 plot(time, M(:, pilDtTIdx), 'r');

 ylabel('d_T (PWM)');
 
subplot(5,1,2)
% plot(time, M(:, pwmDaIdx), 'r');
plot(time, M(:, pilDaTIdx), 'r');
 ylabel('d_A(PWM)');

subplot(5,1,3)
 %plot(time, M(:, pwmDrIdx), 'r');
  plot(time, M(:, pilDrTIdx), 'r');

 ylabel('D_r(PWM)');
 
subplot(5,1,4)
 %plot(time, M(:, pwmDeIdx), 'r');
  plot(time, M(:, pilDeTIdx), 'r');

 ylabel('d_E (PWM)');

 subplot(5,1,5)
 plot(time, M(:, sysModeIdx), 'r');
 ylabel('Nav Mode');
 xlabel ('Time (s)');

eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;

%% Scaled Data

 figure(fig_ct)
% 
% subplot(4,3,1)
%  plot(time, M(:, scaAxIdx), 'r');
%  ylabel('Accels (m/s)');
% 
% subplot(4,3,2)
%  plot(time, M(:, scaAyIdx), 'r');
% 
% subplot(4,3,3)
%  plot(time, M(:, scaAzIdx), 'r');
% 
% subplot(4,3,4)
%  plot(time, M(:, scaGxIdx)*180/pi, 'r');
%  ylabel('Gyros (deg/s)');
% 
% subplot(4,3,5)
%  plot(time, M(:, scaGyIdx)*180/pi, 'r');
% 
% subplot(4,3,6)
%  plot(time, M(:, scaGzIdx)*180/pi, 'r');
%  
% subplot(4,3,7)
%  plot(time, M(:, scaMxIdx), 'r');
%  ylabel('Mags (mG)');
% 
% subplot(4,3,8)
%  plot(time, M(:, scaMyIdx), 'r');
% 
% subplot(4,3,9)
%  plot(time, M(:, scaMzIdx), 'r');
%  
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
 
% subplot(4,3,10)
%  plot(time, M(:, rawPitIdx), 'r');
%  ylabel('Dynamic Pressure (counts)');
% 
% subplot(4,3,11)
%  plot(time, M(:, rawBarIdx), 'r');
%  ylabel('Static Pressure (counts)');
%  
% subplot(4,3,12)
%  plot(time, M(:, rawTheIdx), 'r');
%  ylabel('Air Temperature (counts)');
% eval(['print -depsc  '  num2str(fig_ct)]);
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
 
% subplot(3,1,2)
%  plot(time, M(:, visVolIdx), 'r');
%  axis tight
%  ylabel('Battery Voltage (Volts)');
% 
% subplot(3,1,3)
%  plot(time, M(:, visReaIdx), 'r');
%  axis tight
%  ylabel('Current Consumption(Amps)');
%  
 eval(['print -depsc  '  num2str(fig_ct)]);
fig_ct = fig_ct + 1;
%% Camera
% figure(fig_ct);
% 
% subplot(3,1,1)
%  plot(time, M(:, ptzZoomIdx), 'r');
%  axis tight
%  ylabel('Zoom');
%  
% subplot(3,1,2)
%  plot(time, M(:, ptzPanIdx), 'r');
%  axis tight
%  ylabel('Pan (Degrees)');
% 
% subplot(3,1,3)
%  plot(time, M(:, ptzTiltIdx), 'r');
%  axis tight
%  ylabel('Tilt (Degrees)');
%  
%  eval(['print -depsc  '  num2str(fig_ct)]);
% fig_ct = fig_ct + 1;
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

%% 
figure(fig_ct);

subplot(3,1,1)
 plot(time, M(:, rawGxIdx), 'r.');
 ylabel('Accels (counts)');

subplot(3,1,2)
 plot(time, M(:, rawGyIdx), 'r.');

subplot(3,1,3)
 plot(time, M(:, rawGzIdx), 'r.');
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
