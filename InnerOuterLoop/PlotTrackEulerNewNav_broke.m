%% Extract Files from multidimensional format
x = X(:);
y = Y(:);
z = Z(:);

vn = Vxyz(:,1);%Vned(:,1);
vn = vn(:);
ve = Vxyz(:,2);%Vned(:,2);
ve = ve(:);
vd = Vxyz(:,3);%Vned(:,3);
vd = vd(:);

% housekeeping variables
 l = 1;
 i =  length(vn);%length(eta);
 figct = 1;
 
% Turn the Pause On
pauseOn = 1;

%% Start the Main loop
figure(figct);
%figure(2);
clf;


% Plot the waypoints
plot(Ypoints,Xpoints,'k','LineWidth',0.5);
hold on

%R = 98.32;%(1.5*U_comm^2)/maxAcc;
tp = 0:0.1:2*pi;

if (latlong_WP == 1)
  % This is a square around the east field at UCSC.
 
  % Find the distances between each point and the first point. The first
  % lat/lon point will essentially become (0,0). Information from this
  % website: 'http://www.meridianworlddata.com/Distance-calculation.asp'
  % used extensively. 
  lat_dist = [];
  lon_dist = [];
  
  % first point, delineated as (0,0).
  %changed to use gsLocation of the gsLocation.m file
  first_lat = GS_location(2);%Xpoints(1,1);
  first_lon = GS_location(3);%Ypoints(1,1);
  
  % miles to meters conversion factor
  miles_to_meter = 1609.344;
 
  for i=1:(length(Xpoints))
      current_lat = Xpoints(1,i);
      current_lon = Ypoints(1,i);
      
      current_dist_lat = miles_to_meter * (69.1 * (current_lat - first_lat));
      current_dist_lon = miles_to_meter * (69.1 * (current_lon - first_lon) * cos(first_lat/57.3));
      
      lat_dist    = [lat_dist current_dist_lat];
      lon_dist    = [lon_dist current_dist_lon];    
  end
  
  % Place the proper points in their proper data structures.
  Xpoints_LL = [lat_dist];
  Ypoints_LL = [lon_dist];
  for nn = 1:wpCount
    text(Ypoints_LL(nn)+25,Xpoints_LL(nn)+25,num2str(nn))
    %plot(Ypoints(nn)+R*cos(tp), Xpoints(nn)+R*sin(tp),'k');
  end
    plot([Ypoints_LL(end) Ypoints_LL(1)],[Xpoints_LL(end) Xpoints_LL(1)],'k','LineWidth',0.5);
    plot(Ypoints_LL,Xpoints_LL,'sk');
else     
    for nn = 1:wpCount
    text(Ypoints(nn)+25,Xpoints(nn)+25,num2str(nn))
    %plot(Ypoints(nn)+R*cos(tp), Xpoints(nn)+R*sin(tp),'k');
    end
    plot([Ypoints(end) Ypoints(1)],[Xpoints(end) Xpoints(1)],'k','LineWidth',0.5);
    plot(Ypoints,Xpoints,'sk');
end




title('Position and L2 Vector');

 %plot IP if z value != -999
if (~isempty(IP))
    plot( IP(1,2), IP(1,1),'ks','MarkerSize',15,'MarkerFaceColor','g');
end
 %plot BP if z value != -999
if (~isempty(BP))
    plot( BP(1,2), BP(1,1),'ko','MarkerSize',15,'MarkerFaceColor','r');
end

%Plot the circle turns
% plotCirc
% % 
axis equal;
idx = 1:25:i;

for j=1:50:i-1
    plot(y(j),x(j) ,'-s','MarkerSize',3);
%     if (mod(j-1,20) == 0)
%         %plot(BP(j,2), BP(j,1), 'rs','MarkerSize',3);
%         if (AimPt(j,3) ~= -999)
%             plot(AimPt(j,2), AimPt(j,1),'gs','MarkerSize',3);
%         end
%     end
%      plot the velocity vector
%      plot ([y(j) y(j)+ve(j)], [x(j) x(j)+vn(j)], 'r');
%      
%     %plot the L2 vector
%      if j > 10 % REN 05/24/10 && L2Enabled(j) == 1
%          plot ([y(j) y(j)+L1(j,2)], [x(j) x(j)+L1(j,1)], 'm-');         
%          plot ([y(j) y(j)+AimPointVect(j,2)], [x(j) x(j)+AimPointVect(j,1)], 'm-');
%      end
%     % plot N exagerated (multiplied by 20)
%      % plot ([y(j) y(j)+20*N(j,2)], [x(j) x(j)+20*N(j,1)], 'c-');
%       
% %       %pause the animation
     if pauseOn == 1
        if (mod(j-1,1000)==0)
             pause(0.3);
        end
     end
 end
 plot(y,x ,'-','LineWidth',2.5);
 xlabel('Y(m)');
 ylabel('X(m)');
 grid on;
 hold off
 
%  eval(['print -depsc  '  num2str(figct) '_'  datestr(now,1) '_' ... 
%      datestr(now,'HH') '_' datestr(now,'MM') '_' datestr(now,'SS')]);

% +++++++++++++++++++++++++++++++++++++++++
 figct = figct + 1;
 %%  Collect the rest of the values
% %time 

%%
timePl = ctrlCmds.time;

% Collect Euler Data
phiPl = EulerData.signals(1,1).values(:,1);
phi_cPl = EulerData.signals(1,1).values(:,2);
thetaPl = EulerData.signals(1,2).values(:,1);
theta_cPl = EulerData.signals(1,2).values(:,2);
psiPl = EulerData.signals(1,3).values(:,1);

% Collect Airspeed
umPl = AirspeedPsiDot.signals(1,1).values(:,1);
um_cPl = AirspeedPsiDot.signals(1,1).values(:,2);

%collect PsiDot
psidotPl = AirspeedPsiDot.signals(1,2).values(:,1);
psidot_cPl = AirspeedPsiDot.signals(1,2).values(:,2);

% Collect Control Commands and after servos
decPl = ctrlCmds.signals(1,1).values(:,1);
deservoPl = ctrlCmds.signals(1,1).values(:,2);

dacPl = ctrlCmds.signals(1,2).values(:,1);
daservoPl = ctrlCmds.signals(1,2).values(:,2);

drcPl = ctrlCmds.signals(1,3).values(:,1);
drservoPl = ctrlCmds.signals(1,3).values(:,2);

dtcPl = ctrlCmds.signals(1,4).values(:,1);
dtservoPl = ctrlCmds.signals(1,4).values(:,2);

% Wind Data
windTime = windData(:,1);
xw = windData(:,2);
yw = windData(:,3);
zw = windData(:,4);
%% Plot the values
% % Euler Angles
% figure(figct)
% subplot(4,1,1)
%   plot(eta.time,eta.signals.values,'b');
%   xlabel('Time(s)');
%   ylabel('\eta (deg)');
%   %legend('Measured','Commanded');
%   axis tight
%   grid on;
% subplot(4,1,2)
%   plot(timePl,phiPl*180/pi,'b',timePl,phi_cPl*180/pi,'r');
%   xlabel('Time(s)');
%   ylabel('\phi (deg)');
%   %legend('Measured','Commanded');
%   axis tight
%   grid on;
% subplot(4,1,3)
%   plot(timePl,thetaPl*180/pi,'b',timePl,theta_cPl*180/pi,'r');
%   xlabel('Time(s)');
%   ylabel('\theta (deg)');
%   axis tight
%   grid on;
% subplot(4,1,4)
%   plot(timePl,psiPl*180/pi,'b');
%   xlabel('Time(s)');
%   ylabel('\psi (deg)');
%   axis tight
%   grid on;
%%%%%%%%%%%%%% Manor A. %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Air Speed
%Throttle 
% figct = figct + 1;
figure(figct)
%timePl = ctrlCmds.time;
subplot(3,2,1)
  plot(timePl,umPl ,'b',timePl, um_cPl,'r');
%  xlabel('Time(s)');
  ylabel('Air Speed');
  %legend('Measured','Commanded');
  axis tight
  grid on;
 subplot(3,2,3)
  plot(timePl,thetaPl*180/pi,'b',timePl,theta_cPl*180/pi,'r');
%  title('Pitch');
%  xlabel('Time(s)');
  ylabel('Pitch \theta [deg]');
  axis tight
  grid on;
subplot(3,2,4)
  plot(timePl,dtcPl ,'r',timePl,dtservoPl,'b');
  xlabel('Time [s]');
  ylabel('Throttle');
  axis tight
  grid on;
 
 subplot(3,2,2)
  plot(timePl,z,'b',timePl,h_c,'r');
%  xlabel('Time(s)');
  ylabel('Altitude');
  %legend('Measured','Commanded');
  axis tight
  grid on;
  
 subplot(3,2,5)
  plot(timePl,decPl ,'r',timePl,deservoPl,'b');
%  xlabel('Time(s)');
  ylabel('Elevator');
  axis tight
  grid on;
  
 %Altitude
 %Pitch
 %Elevator
 %Throttle
% figct = figct + 1;
% figure(figct)
% subplot(4,1,1)
%   plot(timePl,z,'b',timePl,h_c,'r');
% %  xlabel('Time(s)');
%   ylabel('Altitude');
%   %legend('Measured','Commanded');
%   axis tight
%   grid on;
%  subplot(4,1,2)
%   plot(timePl,thetaPl*180/pi,'b',timePl,theta_cPl*180/pi,'r');
% %  title('Pitch');
% %  xlabel('Time(s)');
%   ylabel('Pitch \theta [deg]');
%   axis tight
%   grid on;
% subplot(4,1,3)
%   plot(timePl,decPl ,'b',timePl,deservoPl,'r');
% %  xlabel('Time(s)');
%   ylabel('Elevator');
%   axis tight
%   grid on;
%   
%   subplot(4,1,4)
%   plot(timePl,dtcPl ,'b',timePl,dtservoPl,'r');
%   xlabel('Time(s)');
%   ylabel('Throttle');
%   axis tight
%   grid on; 
