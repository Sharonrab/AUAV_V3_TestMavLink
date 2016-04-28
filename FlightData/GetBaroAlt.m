function H = GetBaroAlt(P) 
basePressure = 98192.64;
normP = (P - basePressure)/basePressure;
a2 = 9.350253e-2;
a1 = -1.888933e-1;
a0 = 2.180313e-5;


baseTemp = 288.215;
LapseRate = -0.00198122;

baseAlt = 89 + 170;
Ft2Meters = 0.3048;%[m]
H = (a2*(normP.^2) + a1*normP + a0)*(baseTemp/(-LapseRate))*Ft2Meters + baseAlt;