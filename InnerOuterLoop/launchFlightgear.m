% Runs the InnerOuter sim with flightgear
LAUNCH_FLIGHTGEAR=1;
FLIGHTGEAR_HOST = '127.0.0.1'; % flightgear host address
FLIGHTGEAR_UDP_FDM_PORT = 5502; % udp port for flightgear FDM data
FLIGHTGEAR_UDP_SLUGS_PORT = 56832; % udp port for drawing slugs specific flightgear content (waypoints, vectors, ect...)
FLIGHTGEAR_UDP_SLUGS_RATE = 25; %(hz) update rate for slugs udp stream
LOAD_DELAY=18; %(s) for flightgear to load
FLIGHTGEAR_L2_RATE = 10; % (hz) rate to send L2 data to matlab (must be a multiple of SampleT (0.005) and apSampleTime (0.1) 

% Start UDP connection for slugs specific rendering
if exist('u','var') && strcmp(class(u),'udp') && strcmp(u.status,'open')
    fclose(u); delete(u);
    disp('Closed and removed old flightgear UDP stream.');
end
u = udp(FLIGHTGEAR_HOST,FLIGHTGEAR_UDP_SLUGS_PORT);
clear err;


try
    
    % Launch initial scripts for waypoints
    %run('innerOuterInit')
    %run ..\apConfiguration\simulationWPFile.m
    %run ..\apConfiguration\gsLocation.m
    %run ..\apConfiguration\NavData.m
    Start_pos = [GS_location(2),GS_location(3), Pos_0(1,3)*3.2808  ];

    % Launch Flightgear
    if LAUNCH_FLIGHTGEAR
        run ..\apConfiguration\startFlightgear.m
        pause(LOAD_DELAY);
    end
    
    % Open udp stream for communication with flightgear
    %u = udp('127.0.0.1',PORT);
    fopen(u);
    pause(1);
    
    % Send waypoints to flightgear for drawing
    run ..\apConfiguration\sendWPsToFlightgear.m
    pause(1);
    
    
catch err
    % Do nothing
    disp('An error occured while running inner outer loop sim with flightgear.');
end

% Stop flightgear update thread
%delete(job);
%disp('Ended flightgear update stream.');

% Close udp port
if exist('err') && exist('u','var') && strcmp(class(u),'udp') && u.strcmp(u.status,'open')
    try
    fclose(u);
    delete(u);
    clear u;
    fprintf('\n\nClosed udp stream to flightgear due to error.\n');
    catch err2
    end
else
    disp('Flightgear slugs udp stream left open. Use: fclose(u); delete(u); clear u;');
end

if  exist('err') && strcmp(class(err),'MException')
    rethrow(err);
else
    disp('Ready for simulation.');
end