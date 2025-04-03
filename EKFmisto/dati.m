gradi = pi/180;

DISEGNA = 0; % mettere 1 per vedere figure

Nstep = 1;

sigmaRange = 5; %cm
sigmaBearing = 5*pi/180; % gradi*pi/180 = rad
% sigmaBussola = 1*pi/180;


L = 400; % lunghezza lato ambiente (quadrato)

if nTag == 3
    cTag = [200 300;
        100 120;
        300 120];
elseif nTag == 2
    cTag = [200 300;
        100 120];
elseif nTag == 1
    cTag = [200 300];
else
    cTag = [200 300;
        100 120;
        300 120];
end