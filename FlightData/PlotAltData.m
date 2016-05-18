figure
subplot(3,1,1) 
 plot(time, data(:, airStaIdx));
 ylabel('Static Pressure (Pa)');
 
 subplot(3,1,2)
 plot(time, data(:, posZIdx));
 ylabel('Z (m)');
 
 subplot(3,1,3)
 plot(time, data(:, gpsHeiIdx),'r');%'.' ,
 ylabel('GPS Height (m)');
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 BaroAlt = GetBaroAlt(data(:, airStaIdx));
plot(time,BaroAlt)
hold on
plot(time, data(:, gpsHeiIdx),'r')
hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
 figure
 plot(time, (M(:, gpsHeiIdx)-min(M(:, gpsHeiIdx)))/(max(M(:, gpsHeiIdx))-min(M(:, gpsHeiIdx))));
 hold on
 plot(time, (M(:, airStaIdx)-max(M(:, airStaIdx)))/(min(M(:, airStaIdx))-max(M(:, airStaIdx))), 'r');
 hold off
 title('Normalized GPS Alt (blue) vs Norm Static pressure (red)');

 subplot(2,1,1)
 plot(time, M(:, rawBarIdx), 'r')
 ylabel('Static Pressure counts')
 subplot(2,1,2)
 plot(time, M(:, airStaIdx));
 ylabel('Static Pressure (Pa)');
 
 

 