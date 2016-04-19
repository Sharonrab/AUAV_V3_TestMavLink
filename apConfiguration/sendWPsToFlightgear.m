%SENDWPsTOFLIGHTGEAR Sends all waypoints to flightgear
%   Sends all waypoints in waypoint array to flightgear via the UDP stream u
%   that must be defined in the workspace. Waypoint are read from Xpoints, 
%   Ypoints, and Zpoints arrays which hold the latitude, longitude, and altitude
%   (in meters) of each each waypoint. wpCount must also be defined as the
%   number of waypoints to send.
%

if ~exist('Xpoints','var') || ~exist('Ypoints','var') || ~exist('Zpoints','var') || ~isvector(Xpoints) || ~isvector(Ypoints) || ~isvector(Zpoints) || ~exist('wpCount','var')
    error('Simulation WP file was never loaded.');
end
if ~exist('u','var') || ~strcmp(class(u),'udp')
    error('A udp socket ''u'' was never created.');
end

if wpCount < 1
    disp('No waypoints to send to flightgear.');
    return;
end

disp('Sending waypoints to flightgear...');

% Load waypoints and clear old ones
sendWPCommandToFlightgear(u,'@CLEAR'); % clear existing waypoints
pause(0.2);
sendWPCommandToFlightgear(u,'@DELETE0'); % clear airport
pause(0.2);

% Send each waypoint
for i=1:wpCount
    sendWPToFlightgear(u,Xpoints(i),Ypoints(i), Zpoints(i));
    pause(0.2);
end

% Send WP1 to close loop
sendWPToFlightgear(u,Xpoints(1),Ypoints(1), Zpoints(1));
pause(0.2);

sendWPCommandToFlightgear(u,'waypoint'); % draws all waypoints and legs
pause(0.2);


fprintf('Sent %d waypoints to flightgear to be drawn.',wpCount + 1);
