clear
close all;
GetDataIndexes  %load indices
load 'my_csv_log_file.mat'
%load 'my_csv_log_file'
%time = (data(:, timeStampIdx) - data(1,timeStampIdx))*0.01;
k =find(data(:,timeStampIdx)<6050);
sz = size(k);
if sz(:,1)==0
    time = data(:,timeStampIdx);
else
    time = data(1:k(1)-1,timeStampIdx);
    for i=1:sz(1)-1
    time = [time; data(k(i):k(i+1)-1,timeStampIdx)+data(k(i)-1,timeStampIdx)];
    %time = [data(1:110,1); data(111:376,1)+data(110,1) ;data(377:end,1)+data(376,1)+data(110,1) ];
    end
    time = [time; data(k(i+1):end,timeStampIdx)+data(k(i)-1,timeStampIdx)+data(k(i+1)-1,timeStampIdx)];

end
time = (time(:, timeStampIdx) - time(1,timeStampIdx))*0.01;

plot(time(:), data(:, sysModeIdx), 'r');
% navWp1Idx
% sysModeIdx
% sysNavIdx
figure
hold on
plot(time(:), data(:, sysNavIdx));
plot(time(:), data(:, sysModeIdx), 'r');
hold off
figure
hold on
plot(time(:), data(:, navWp1Idx));
plot(time(:), data(:, navWp2Idx), 'r');
hold off

figure
hold on
plot(time(:), data(:, navDis2GoIdx));
plot(time(:), data(:, navRemIdx), 'r');
hold off

% logFl1Idx
% figure
% plot(time(:), data(:, logFl1Idx));

% Turn_Lead_D {56} 
% IP_Reach {57} 
% L2 {58} 
% RTB {59} 
% WP_Index {60} 
% WP_Fly {61} 
% WPI_X {62} 
% WPI_Y {63} 
% WPI_Z {64} 
% L2_X {65} 
% L2_Y {66} 
% L2_Z {67} 

