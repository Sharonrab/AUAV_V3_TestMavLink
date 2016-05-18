% =======================================================================
% THIS FILE MUST BE RUN AFTER "NavData.m" TO GET THE MAXIMUM ACCELERATION
% VALUE
% =======================================================================

% This file describes the new navigation program, called L2Plus.
% L2Plus is logic for acquiring and tracking straight lines between waypoints
% by extending the Amidi/Park/Deyst algorithm for acceleration command to the
% regime when the cross track error is bigger than the L2 look-ahead distance.

% There have to be at least 2 waypoints because the logic doesn't know where to
% go after reaching the first waypoint
% 

% =============
% LINE TRACKING
% =============

% The look-ahead point is defined by a the intersection of a vector of length L2, from the vehicle to the path.
% In our forumulation, this distance changes with groundspeed to keep the response dynamics independent
% of groundspeed. We use a variable TStar (T*) which specifies the response time constant. Then
% 
% L2 == max(1,TStar*Groundspeed)

% The max operator is used to accomodate boats and other vehicles that can have zero groundspeed.

% L2Plus continues the concept of steering toward an aim point. Normally the aim point is the intersection 
% of the L2 vector and the desired path, the Amidi/Park/Deyst logic. This point becomes
% undefined when the cross track error is greater than the (scalar) L2. 

% Instead, we use an aim point defined by the intercept angle. The aim point is the down-track 
% distance determined by
% 
%     abs(DistXtrack)/tanIntercept
%
% where DistXtract is the cross track distance, and tanIntercept is the
% tangent of the intercept angle. This could be a very big distance during
% initial intercept when the cross track error is large, so an upper bound
% is placed on it, a bound proportional to L2 (constant MaxDP_Star) .

% When the cross track error is within L2 of the track, we still use the
% intercept angle to limit the intercept angle to mitigate overshoot.
% 
% If the vehicle is beyond the switchpoint it will keep going away from the switchpoint
% since the logic relies on the waypoint switching logic to change to the
% next waypoint to create a switchpoint ahead of the vehicle

% The code snippet to compute the down path distance of the aim point, regardless
% of the cross track error, is below. 
% 
%         MinDownPath = single ( min( abs(Pe_y)/tanIntercept, MaxDwnPthStar*L2)); 
%         if ( abs(Pe_y) > L2 )
%             L2DownPath = MinDownPath;
%         else
%             within L2 of T but limit angle of L2 wrt T
%             L2DownPath = (max(MinDownPath, (sqrt( L2^2 - Pe_y^2 ))));
%         end
%          % vector from Pwp1 to aim point on T1
%          L2ToT = single(( Dist2WP1 - L2DownPath )*(-T1) - Pwp1);
%          L2Vec = single(L2*( L2ToT/norm(L2ToT) )); % limit size to L2
% where Pe_y is the cross track error, Dist2WP1 is the along-track distance to WP1,
% Pwp1 is the vehicle position relative to WP1, T1 is the unit path vector from WP0 to WP1
% 
% The initial conditions may have the velocity vector headed away from the first active waypoint. 
% A maximum bank turn is called for, but there are two options for turning:
%     1. toward the desired track. this minimizes the cross track deviation, but send the vehicle
%         away from the active waypoint
%     2. turn toward the L2 vector. This leads to larger cross track deviations but heads toward
%         the active waypoint
% The flag "Turn2Track", if true, enables option 1, otherwise option 2.
%
% Turn2Track is disabled if the vehicle is beyond the waypoint switch point because the
% logic will diverge.

% =============
% INITIAL POINT
% =============

% sometimes it is desired to enter into the array of waypoints in a consistent state (position & velocity) regardless 
% of the initial conditions when the guidance system is turned on. To do this, we borrow a concept from 
% instrument flying in which all landing procedures have a well defined point to which all aircraft first fly
% before proceeding to the runway. 
% 
% We do the same here, by establishing a point, called the Initial Point, or IP, on the line determined by the 
% first two waypoints in the waypoint array. This point is at a distance IPStar*L2 before the first waypoint.
% This becomes the first aimpoint for the L2Plus logic, and the vehicle proceeds toward the IP. When the vehicle gets within 0.1*L2 
% of the IP, the IP logic is turned off and normal L2Plus takes over.
% 
% This is very similar to the very first logic which is being replaced by L2Plus. That logic established a fictitious waypoint
% at the vehicle initial position, and the vehicle tracked the line between this WP and the first WP in the array of WPs.
% By contrast, there is no line established when L2Plus is aiming at the IP. 

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

% ===============================
% Control Saturation
% ===============================

% For both line tracking and homing, the logic is set up to saturate when
% eta, the angle between the groundspeed vector and the aim poiont
% direction reaches a limit. This angle limit, on the order of 45 ro 60
% degrees, ensures a faster response time without sacrificing system
% transient response.

% We set the limits for UAVs based on the responsee time of the roll
% autopilot. A good measure of this is the variable "TurnLeadTime" which is
% used to start the turn before reaching the waypoint switch point. We set
% the maximum eta for normal guidance to be the minimum of 90 degrees or
% the angle through which the auto can turn at half the maximum
% acceleration (an approximate average acceleration as the bank angle goes from maximum
% to zero.

% The average turn rate during this maneuver is
%     avgPsiDot = (maxAcc/2)/CommandAirSpeed
%     TurnAngle = min(pi/2,TurnLeadTIime*avgPsiDot)

%======== Parameters ===========================
% TRACKING

% Time constatn of system
TStar = single(4);

% Lead time in seconds to start turn before reaching circle tangent point
TurnLeadTime = 4;

% Limit to downpath distance = MaxDwnPthStar*L2
MaxDwnPthStar = 1;

% Intercept angle
tanIntercept = tan( 30*pi/180 );

% flag to turn toward track (!= 0) or toward L2 (== 0) 
Turn2Track = 0;

% INITIAL POINT
% flag to turn on initial point logic, 0 for no IPS
InitialPoint = 0;

% L2 multiplier to determine distance of IP
% from the first "from" waypoint. Setting this to 0
% causes the vehicle to go directly toward the first WP if InitialPoint is
% enabled
IPStar = 2;


% RETURN TO BASE
% Base position. Vehicle will return here if RTB flag is true using the same homing logic. 
% When reaching the base the logic remains active, and the vehicle will circle around the base point.
BasePoint =  [-100; 200; 230];


% MAXIMUM ETA FOR CONTROL SATURATION
% U_comm and maxAcc is defined in NavData.m
maxEta = single(min(pi/2,TurnLeadTime*((maxAcc/2)/U_comm)));
maxEta = single(90*pi/180);
% maxEta = 20*pi/180;