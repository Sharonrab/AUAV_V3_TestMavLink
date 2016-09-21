function [serialPortString] = pickSerialPort()
h = figure;
serialInfo = instrhwinfo('serial');
serialPortString = '';

if ~isstruct(serialInfo) || length(serialInfo.AvailableSerialPorts) < 1
    error('Could not find any COM ports to choose from.');
end


hm1 = uicontrol('units','normalized','position',[0.18,0.7,0.45,0.05],...
    'style','popup','string',serialInfo.AvailableSerialPorts,...
    'value',1);
  
uicontrol('Style','text','units','normalized','position',[0.07,0.8,0.7,0.1],...
    'String','Multiple serial ports are available. Please choose the port for the serial to UDP device:','FontUnits', 'normalized');

uicontrol('style', 'pushbutton', 'string', 'Ok', 'units','normalized',...
    'position', [0.31, 0.6, 0.1, 0.05], 'callback', @pressOk);
uicontrol('style', 'pushbutton', 'string', 'Cancel', 'units','normalized',...
    'position', [0.45, 0.6, 0.1, 0.05], 'callback', @pressCancel);



     function pressOk(hObj,~)
        choice = get(hm1,'value'); 
        serialPortString = serialInfo.AvailableSerialPorts{choice};
        close(h);
     end
         
     function pressCancel(hObj,~)
         close(h);
     end
 
 
 waitfor(h);
end