%  Initial Conditions in ENU (all vector data is represented as a column
%  vectors)
% Pos_0   = [-250; 50; 230]';     % Initial position vector (m)
Pos_0   = [0 ;0 ; 110 + baseHeight]';     % Initial position vector (m) starts at ground station
Euler_0 = [0*pi/180; 0; 0*pi/180]';   % Initial Euler angles    (rad)
Omega_0 = [0; 0; 0]';            % Initial Omega           (rad/s)
PQR_0   = [0;0;0]';      % Initial Omega           (rad/s)
Vb_0    = [ 17; 0;0]';   % Initial body-velocity vector (m/s)

%% Nav Values

U_comm = single(17);

% this requires Environment.m amd limits.m to be loaded first
% factor of 1.1 on max ground speed is for contingencies 
maxAcc = single(ISA_g*tan(bankCommandLimit));
MinTurnRadius = single((1.1*(U_comm + windBase)^2)/maxAcc);

