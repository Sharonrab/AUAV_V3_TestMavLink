%% PWM Conversions and viceversa for the PAN - TILT unit

%% Curve fitting for PWM . Conversion from Radians to PWM and vice versa
% Pan
% The Values are as follows:
rad = [90 80 70 60 50 40 30 20 10 0 -10 -20 -30 -40 -50 -60 -70 -80 -90]*pi/180;
pwm = [362 442 506 578 634 714 778 842 914 978 1034 1114 1178 1250 1314 1386 1450 1516 1586];
panMax = max(pwm);
panMin = min(pwm);

P =  polyfit(rad, pwm,1);
mpan = P(1);
bpan = P(2);

P =  polyfit(pwm, rad,1);
mPWMpan = P(1);
bPWMpan = P(2);


% Tilt
% The Values are as follows:

rad = [-62.2 -45 -40 -20 -15 -10 0]*pi/180;
pwm = [874 986 1018 1138 1170 1202 1266];

tiltMax = max(pwm);
tiltMin = min(pwm);

P =  polyfit(rad, pwm,1);
mtilt = P(1);
btilt = P(2);

P =  polyfit(pwm, rad,1);
mPWMtilt = P(1);
bPWMtilt = P(2);

%% Not used, placed here for compilation compatibility
% Roll
% The Values are as follows:
% -20 -> 2000
% +20 -> 4725

rad = [17.5 0  -20.1]*pi/180;
pwm = [857 977 1097];

P =  polyfit(rad, pwm,1);
mroll = P(1);
broll = P(2);

P =  polyfit(pwm, rad,1);
mPWMroll = P(1);
bPWMroll = P(2);
