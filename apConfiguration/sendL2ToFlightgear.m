function sendL2ToFlightgear(u, lat,lon)
%SENDWPTOFLIGHTGEAR
%   Sends an L2+ vector ending at the given latitude and longitude to
%   flightgear to be drawn. The vector starts at the aircraft's position.

fwrite(u,sprintf('\t\t%f\t%f\t1\n',lat,lon));

end

