%% Extract Files from multidimensional format
x = X(:);
y = Y(:);
z = Z(:);

vn = Vned(:,1);
vn = vn(:);
ve = Vned(:,2);
ve = ve(:);
vd = Vned(:,3);
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
for nn = 1:wpCount
    text(Ypoints(nn)+25,Xpoints(nn)+25,num2str(nn))
end
plot([Ypoints(end) Ypoints(1)],[Xpoints(end) Xpoints(1)],'k','LineWidth',0.5);
plot(Ypoints,Xpoints,'sk');
title('Position and L2 Vector');

 %plot IP if z value != -999
ZZ = find(IP(:,3)~= -999,1);
if (~isempty(ZZ))
    plot( IP(ZZ,2), IP(ZZ,1),'ks','MarkerSize',15,'MarkerFaceColor','g');
end
 %plot BP if z value != -999
ZZ = find(BP(:,3)~= -999,1);
if (~isempty(ZZ))
    plot( BP(ZZ,2), BP(ZZ,1),'ko','MarkerSize',15,'MarkerFaceColor','r');
end

%Plot the circle turns
% plotCirc
% % 
axis equal;
idx = 1:25:i;

for j=1:10:i-1
    plot(y(j),x(j) ,'-s','MarkerSize',3);
    if (mod(j-1,20) == 0)
        plot(BP(j,2), BP(j,1), 'rs','MarkerSize',3);
        if (AimPt(j,3) ~= -999)
            plot(AimPt(j,2), AimPt(j,1),'gs','MarkerSize',3);
        end
    end
     %plot the velocity vector
%      plot ([y(j) y(j)+ve(j)], [x(j) x(j)+vn(j)], 'r');
     
     %plot the L2 vector
%      if j > 10 % REN 05/24/10 && L2Enabled(j) == 1
%         plot ([y(j) y(j)+L1(j,2)], [x(j) x(j)+L1(j,1)], 'm-');
%      end
     % plot N exagerated (multiplied by 20)
%       plot ([y(j) y(j)+20*N(j,2)], [x(j) x(j)+20*N(j,1)], 'c-');
      
     %pause the animation
     if pauseOn == 1
        if (mod(j-1,200)==0)
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
% %%  Collect the rest of the values
% %time 

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
% Euler Angles
figure(figct)
subplot(4,1,1)
  plot(eta.time,eta.signals.values*180/pi,'b');
  xlabel('Time(s)');
  ylabel('\eta (deg)');
  %legend('Measured','Commanded');
  axis tight
  grid on;
subplot(4,1,2)
  plot(timePl,phiPl*180/pi,'b',timePl,phi_cPl*180/pi,'r');
  xlabel('Time(s)');
  ylabel('\phi (deg)');
  %legend('Measured','Commanded');
  axis tight
  grid on;
subplot(4,1,3)
  plot(timePl,thetaPl*180/pi,'b',timePl,theta_cPl*180/pi,'r');
  xlabel('Time(s)');
  ylabel('\theta (deg)');
  axis tight
  grid on;
subplot(4,1,4)
  plot(timePl,psiPl*180/pi,'b');
  xlabel('Time(s)');
  ylabel('\psi (deg)');
  axis tight
  grid on;

