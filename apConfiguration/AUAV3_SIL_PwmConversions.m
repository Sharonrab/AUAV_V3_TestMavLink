%% Curve fitting for PWM . Conversion from Radians to PWM and vice versa
% Rudder
% The Values are as follows:
% -20 -> 2700
% -10 -> 3250
% -5  -> 3475
% 0   -> 3750
% +5  -> 3950
% +10 -> 4200
% +20 -> 4725

rad = [20 0 -20]*pi/180;
pwm = [9490 13154 16815];
P =  polyfit(rad, pwm,1);
mdr = P(1);
bdr = P(2);

P =  polyfit(pwm,rad,1);
mPWMdr = P(1);
bPWMdr = P(2);

% Aileron
% The Values are as follows:
% -20 -> 2450
% -10 -> 3150
% -5  -> 3420
% 0   -> 3700
% +5  -> 3975
% +10 -> 4250
% +20 -> 4900

rad = [15 0 -15]*pi/180;
pwm = [9490 13154 16815];
P =  polyfit(rad,pwm,1);
mda = P(1);
bda = P(2);

P =  polyfit(pwm, rad,1);
mPWMda = P(1);
bPWMda = P(2);

% Elevator
% The Values are as follows:
% -15 -> 2750
% -10 -> 3135
% -5  -> 3425
% 0   -> 3700
% +5  -> 4125
% +10 -> 4450
% +15 -> 4850

rad = [-10 0 10]*pi/180;
pwm = [9490 13154 16815];
P =  polyfit(rad,pwm,1);
mde = P(1);
bde = P(2);

P =  polyfit(pwm,rad,1);
mPWMde = P(1);
bPWMde = P(2);


% Throttle
% 0  ->  2500
% 1  ->  4850

rad = [0 1];
pwm = [9386 16815];%Why the scale was ???/10
P =  polyfit(rad, pwm, 1);
mdt = P(1);
bdt = P(2);

P =  polyfit(pwm,rad,1);
mPWMdt = P(1);
bPWMdt = P(2);

%% Curve fitting for IC from Pilot Console. 
% Conversion from IC output to Radians

% Rudder
mICdr = mPWMdr;% was with /2 before
bICdr = bPWMdr;

% Aileron
mICda = mPWMda;% was with /2 before
bICda = bPWMda;

% Elevator
mICde = mPWMde;% was with /2 before
bICde = bPWMde;


%Throttle
mICdt = mPWMdt;% was with /2 before
bICdt = bPWMdt;