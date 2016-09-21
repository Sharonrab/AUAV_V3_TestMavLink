% Runs flightgear
SEND_WAYPOINTS = 0;
FLIGHTGEAR_HOST = '127.0.0.1'; % flightgear host address
FLIGHTGEAR_UDP_FDM_PORT = 5502; % udp port for flightgear FDM data
FLIGHTGEAR_UDP_SLUGS_PORT = 53243; % todo put right port in
FLIGHTGEAR_UDP_SLUGS_RATE = 20;
LOAD_DELAY=18; %(s) for flightgear to load

Start_pos = [GS_location(2),GS_location(3),Pos_0(1,3)+baseHeight  ]; % starting position

try
    % Launch Flightgear
    startFlightgear
    
    
    % Start UDP connection for sending waypoints
    if SEND_WAYPOINTS
        % Acquire waypoints here
        
        pause(LOAD_DELAY);
        if exist('u','var') && strcmp(class(u),'udp')
            fclose(u); delete(u);
            disp('Closed and removed old flightgear UDP stream.');
        end
        
        %u = udp(FLIGHTGEAR_HOST,FLIGHTGEAR_UDP_SLUGS_PORT);

        % Open udp stream for communication with flightgear
        u = udp('127.0.0.1',FLIGHTGEAR_UDP_SLUGS_PORT);
        fopen(u);
        pause(1);

        % Send waypoints to flightgear for drawing
        sendWPsToFlightgear
        pause(1);
    end

    
catch err
    % Do nothing
    disp('An error occured while running inner outer loop sim with flightgear.');
end

% Close udp port
if SEND_WAYPOINTS
    fclose(u);
    delete(u);
    clear u;
    disp('Closed udp stream to flightgear.');
end

if  exist('err') && strcmp(class(err),'MException')
    rethrow(err);
else
    disp('Started flightgear.');
end