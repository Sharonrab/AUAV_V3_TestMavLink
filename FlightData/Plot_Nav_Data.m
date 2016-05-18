
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
figure
plot(time(:), data(:, logFl1Idx));

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

