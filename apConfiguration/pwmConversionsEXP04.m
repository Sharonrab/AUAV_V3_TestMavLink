%% Curve fitting for PWM . Conversion from Radians to PWM and vice versa 
% The pilot consoloe values * 2.5 
% Rudder 
% The Values are as follows: 
rad = [-35.6 0 30.41]*pi/180; 
pwm = [2868 3830 4833]; 
P =  polyfit(rad, pwm,1); 
mdr = P(1); 
bdr = P(2); 
P =  polyfit(pwm,rad,1); 
mPWMdr = P(1); 
bPWMdr = P(2); 

% Aileron 
% The Values are as follows: 
rad = [22.5 0 -23.81]*pi/180; 
pwm = [4535 3853 3210]; 
P =  polyfit(rad,pwm,1); 
mda = P(1); 
bda = P(2); 
P =  polyfit(pwm, rad,1); 
mPWMda = P(1); 
bPWMda = P(2); 

% Elevator 
% The Values are as follows: 
rad = [-19.87 0 34.05]*pi/180; 
pwm = [2783 3795 4748]; 
P =  polyfit(rad,pwm,1); 
mde = P(1); 
bde = P(2); 
P =  polyfit(pwm,rad,1); 
mPWMde = P(1); 
bPWMde = P(2); 

% Throttle 
% 0  ->  2800 
% 1  ->  4625 
rad = [0 1]; 
pwm = [2800 4625]; 
P =  polyfit(rad, pwm, 1); 
mdt = P(1); 
bdt = P(2); 
P =  polyfit(pwm,rad,1); 
mPWMdt = P(1); 
bPWMdt = P(2); 
%% Curve fitting for IC from Pilot Console. 
% Conversion from IC output to Radians 
% Rudder 
mICdr = mPWMdr/2; 
bICdr = bPWMdr; 
% Aileron 
mICda = mPWMda/2; 
bICda = bPWMda; 
% Elevator 
mICde = mPWMde/2; 
bICde = bPWMde; 
%Throttle 
mICdt = mPWMdt/2; 
bICdt = bPWMdt; 

