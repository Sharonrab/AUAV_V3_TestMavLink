%% PWM Conversions and viceversa for the PAN - TILT unit

%% Curve fitting for PWM . Conversion from Radians to PWM and vice versa
% Pan
% The Values are as follows:
rad = [-34 0 34]*pi/180;
pwm = [1250 1026 802];

P =  polyfit(rad, pwm,1);
mpan = P(1);
bpan = P(2);

P =  polyfit(pwm, rad,1);
mPWMpan = P(1);
bPWMpan = P(2);


% Tilt
% The Values are as follows:

rad = [88 45 0]*pi/180;
pwm = [670 905 1200];

P =  polyfit(rad, pwm,1);
mtilt = P(1);
btilt = P(2);

P =  polyfit(pwm, rad,1);
mPWMtilt = P(1);
bPWMtilt = P(2);

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


%% 