% Starts flightgear with the flight dynamics model in network mode for 
% communication with simulink.
DEBUG = 1; % starts a telnet and http interface
DETACH = 1; % open application in a command prompt in the background
FLIGHTGEAR_ROOT = 'C:\Program Files\FlightGear';
FLIGHTGEAR_EXE_PATH = 'C:\Program Files\FlightGear\bin\Win32\fgfs.exe';
AIRPORT = 'CL77'; % bonny doon
AIRCRAFT = 'Cub';
%PORT = 56832; % udp port for custom slugs protocol

extra_args='';

if DEBUG
    extra_args='--httpd=8080 --telnet=5401';
end

if DETACH
    extra_args = [ extra_args ' &' ];
end


% Get task list to check if flightgear is running
[out result ] = system('tasklist');
if (findstr('fgfs.exe',result))
    fprintf('Flightgear is already running.\n');
    return;
end


% Builds command
cmd = sprintf([ ...
    '"%s" --fg-root="%s\\data" '...
    '--fg-scenery="%s\\data\\Scenery;%s\\scenery;%s\\terrasync" '...
    '--lat=%f --lon=%f --altitude=%f --aircraft=%s --control=joystick '...
    '--disable-random-objects --disable-ai-models --disable-ai-traffic '...
    '--prop:/sim/rendering/random-vegetation=false --enable-hud '...
    '--disable-real-weather-fetch --bpp=32 --timeofday=noon '...
    '--fdm=network,%s,5501,%d,5503 '...
    '--generic=socket,in,%d,%s,%d,udp,slugs %s'], FLIGHTGEAR_EXE_PATH,...
    FLIGHTGEAR_ROOT, FLIGHTGEAR_ROOT,FLIGHTGEAR_ROOT,FLIGHTGEAR_ROOT,...
    Start_pos(1),Start_pos(2),Start_pos(3),AIRCRAFT,FLIGHTGEAR_HOST,FLIGHTGEAR_UDP_FDM_PORT,...
    FLIGHTGEAR_UDP_SLUGS_RATE, FLIGHTGEAR_HOST,FLIGHTGEAR_UDP_SLUGS_PORT,extra_args);

if DEBUG
    fprintf('Running FlightGear with: %s\n',cmd);
end

status = system(cmd,'-echo');