
% ===============================
% HOMING - RTB and INITIAL  POINT
% ===============================

% sometimes it is desired to enter into the array of waypoints in a consistent state (position & velocity) regardless 
% of the initial conditions when the guidance system is turned on. To do this, we borrow a concept from 
% instrument flying in which all landing procedures have a well defined point to which all aircraft first fly
% before proceeding to the runway. 
% 
% We do the same here, by establishing a point, called the Initial Point, or IP, on the line determined by the 
% first two waypoints in the waypoint array. This point is at a distance IPStar*L2 before the first waypoint.
% This becomes the aimpoint for the L2Plus logic, and the vehicle proceeds toward the IP. When the vehicle gets within 0.1*L2 
% of the IP, the IP logic is turned off and normal L2Plus takes over.
% 
% This is very similar to the logic which is being replaced by L2Plus. That logic established a fictitious waypoint
% at the vehicle initial position, and the vehicle tracked the line between this WP and the first WP in the array of WPs.
% By contrast, there is no line established when L2Plus is aiming at the IP. 

% ======== Parameters ===========================

% INITIAL POINT
% flag to turn on initial point logic, 0 for no IPS
InitialPoint = 1;

% L2 multiplier to determine distance of IP
% from the first "from" waypoint. Setting this to 0
% causes the vehicle to go directly toward the first WP if InitialPoint is
% enabled
IPStar = 4;

% minimum turning radius is calculated in NavData.m

% RETURN TO BASE
% Base position. Vehicle will return here if RTB flag is true using the same homing logic. 
% When reaching the base the logic remains active, and the vehicle will circle around the base point.
BasePoint = [-200; 200; 230];
