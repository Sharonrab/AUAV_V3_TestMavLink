% Starts the HIL client bridge.
HILCLIENT_EXE_PATH = 'client\HILBridgeClient\Debug\HILBridgeClient.exe';
DETACH = 1;
extra_args='';

if DETACH
    extra_args = [ extra_args ' &' ];
end


% Get task list to check if flightgear is running
[out result ] = system('tasklist');
if (findstr('HILBridgeClient.exe',result))
    fprintf('HIL client bridge is already running.\n');
    return;
end

% Check serial ports
serialPort = 'COM5';
% serialInfo = instrhwinfo('serial');
% ports=length(serialInfo.AvailableSerialPorts);
% if ports > 1
% 	serialPort = pickSerialPort();
% elseif ports == 1
% 	serialPort = serialInfo.AvailableSerialPorts{1};
% end
% 
% if isempty(serialPort)
% 	error('Could not obtain a serial port for the serial to udp device.')
% end

% Builds command
cmd = sprintf('%s --port=%s %s', HILCLIENT_EXE_PATH, serialPort, extra_args);
%fprintf('Running: %s', cmd);

status = system(cmd,'-echo');
