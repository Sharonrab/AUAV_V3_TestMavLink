% This is the file that configures the location of the Ground Station make
% sure that GS_location has 0 altitude since the altitude is read in base
% height. Do not modify this file, since this is under version control,
% rather copy it name it what you want and replace the line where its being
% called in InnerOuterInit.m (simulation) or mcuInit.m (for HIL and
% flight).

% GS Location. East field UCSC location
% GS_location = [0 36.9898376464844 -122.051963806152];
% GS_location = [0 19.0450708 -95.9717882];
% GS_location = [0 36.99 -122.05]; %UCSC
baseHeight = 143.543; % (use this one)
GS_location = [baseHeight 36.988506 -122.055308]; %UCSC (use this one)

%baseHeight = 611.6;
%GS_location = [baseHeight 37.070751835589604 -122.12770032696426];


%baseHeight = 611.6;