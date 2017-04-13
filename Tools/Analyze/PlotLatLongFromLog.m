 
%% Lat Lon Plots 
clear
GetDataIndexes  %load indices
load 'my_csv_log_file.mat'
time = (data(:, timeStampIdx) - data(1,timeStampIdx))*0.01;

subplot(2,1,1)
plot(data(:, gpsLonIdx), data(:,gpsLatIdx));
hold
plot(data(:, gpsLonIdx), data(:,gpsLatIdx), 'r.');
hold 
ylabel('Latitude (deg)');
xlabel('Longitude (deg)');
axis equal
hold off
subplot(2,1,2)
 plot(time(:), data(:, posZIdx), 'r.');
 ylabel('Z (m)');
 xlabel ('Time (s)');
 

