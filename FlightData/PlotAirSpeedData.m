figure
plot(time(8334:end ), (M(8334:end, gpsSogIdx)-min(M(8334:end, gpsSogIdx)))/(max(M(8334:end, gpsSogIdx))-min(M(8334:end, gpsSogIdx))));
hold on
plot(time(8334:end), (M(8334:end, airDynIdx)-min(M(8334:end, airDynIdx)))/(max(M(8334:end, airDynIdx))-min(M(8334:end, airDynIdx))), 'r');
plot(time(8334:end), (M(8334:end, rawPitIdx)-min(M(8334:end, rawPitIdx)))/(max(M(8334:end, rawPitIdx))-min(M(8334:end, rawPitIdx))), 'g');

hold off
title('Normalized GPS SOG (blue), Norm Dynamic pressure (red), Norm dynamic P Raw');



plot(time(:), data(:, rawPitIdx), 'g');
title(' GPS SOG (blue), Dynamic pressure (red), dynamic P Raw');
hold off
figure
plot(time(:), data(:, airTemIdx));
hold on
plot(time(:), data(:, rawTheIdx));
figure
plotyy(time(:), data(:, airTemIdx),time(:), data(:, rawTheIdx));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dynPas =  sqrt(2*data(:, rawPitIdx)/1.225);
GPS_h2Pas =  1.225*(data(:, gpsSogIdx).^2)/2;
Raw2Pas = (data(:, rawPitIdx)-1320.5)*0.81887676;
AirSpd = sqrt(2*Raw2Pas/1.225);
figure
subplot(2,1,1)
plotyy(time(:), data(:, gpsSogIdx),time(:), data(:, rawPitIdx));
title(' GPS SOG (blue), Dynamic pressure Raw');
subplot(2,1,2)
plotyy(time(:), GPS_h2Pas(:),time(:), (data(:, rawPitIdx)-1320.5)*0.81887676);
plotyy(time(:), data(:, gpsSogIdx),time(:), dynPas(:));
plotyy( time(:),GPS_h2Pas(:),time(:), dynPas(:))
figure
plotyy( time(:),GPS_h2Pas(:),time(:), Raw2Pas(:))
plotyy(time(:), data(:, gpsSogIdx),time(:), AirSpd(:));
plotyy( time(:),GPS_h2Pas(:),time(:), data(:, rawPitIdx))
title(' GPS 2 Pascal (blue), Dynamic pressure Raw');


figure
plot(time(: ), data(:, gpsSogIdx));
hold on
plot(time(:), data(:, airDynIdx)/10, 'r');
hold off

figure
plotyy(time(:), data(:, gpsSogIdx),time(:), data(:, airDynIdx)/10);
figure
plotyy(time(:), data(:, gpsSogIdx),time(:), data(:, rawPitIdx));