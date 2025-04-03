d = 25; % distanza tra le due ruote
KR = 0.015;%2.0068e-004;%.015;%.05;%.1;   % Costante KR errori Odometrici ruota destra: uR_e = N(uR, KR |uR|)
KL = KR;%2.0068e-004;%.05;%.1;   % Costante KL errori Odometrici ruota sinistra: uL_e = N(uL, KL |uL|)
deltaRvera = 1;
deltaLvera = 1;

% Spostamenti reali

% nPassi = 3000;
    
% Definizione dei vettori x,y e theta delle coordinate del robot durante la simulazione:
xVett = zeros(nPassi,1);
yVett = zeros(nPassi,1);
thetaVett = zeros(nPassi,1);

clearance = 150;

deltaTheta = zeros(nPassi,1);

xVett(1) = clearance + rand*(L-2*clearance);
yVett(1) = clearance + rand*(L-2*clearance);
thetaVett(1)=360*rand*gradi; % angolo in radianti: gradi e' il fattore di conversione
deltaRho = ones(nPassi,1);

uR = zeros(nPassi,1);
uL = zeros(nPassi,1);

cinque = 150; % angolo max di curva totale
uno = 5; % angolo max di curva in uno step

k = 1;
while k < nPassi

    [lato, distanza] = distanzaBordo(xVett(k),yVett(k),thetaVett(k));

    if distanza < clearance % devo girare
        curvaDaFare = max(uno,rand*cinque); % e' la curva in gradi che deve essere effettuata
        passiCurvaDaFare = round(curvaDaFare/uno); % e' il numero di passi che il robot impieghera' per fare la curva
        gradiPerPasso = curvaDaFare/passiCurvaDaFare;
        kIn = min(k,nPassi);
        kFin = min(k+passiCurvaDaFare,nPassi-1);
        indiceK = kIn:kFin;
        direzione = mod(round(thetaVett(k)/gradi),360); % direzione in gradi tra 1 e 360        
        if lato == 1 % mi sto dirigendo contro il lato sotto
            if direzione>270 % sto puntando verso destra
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso destra
            else 
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso sinistra perche' sto puntando verso sinistra
            end
        elseif lato == 2 % mi sto dirigendo contro il lato destro
            if direzione>180 % sto puntando verso il basso
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso il basso
            else 
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso l'alto perche' sto puntando verso l'alto
            end
        elseif lato == 3 % mi sto dirigendo contro il lato sopra
            if direzione<90 % sto puntando verso destra
                deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso destra
            else 
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso sinistra perche' sto puntando verso sinistra
            end            
        elseif lato == 4 % mi sto dirigendo contro il lato sinistro
            if direzione>180 % sto puntando verso il basso
                deltaTheta(indiceK) = gradiPerPasso*gradi; % giro verso il basso
            else deltaTheta(indiceK) = -gradiPerPasso*gradi; % giro verso l'alto perche' sto puntando verso l'alto
            end
        end 
        for k = kIn:kFin
            uR(k) = (deltaRho(k) + (d/2)*deltaTheta(k));
            uL(k) = (deltaRho(k) - (d/2)*deltaTheta(k));

            xVett(k+1)= xVett(k)+ ((uR(k)+uL(k))/2)*cos(thetaVett(k));
            yVett(k+1)= yVett(k)+ ((uR(k)+uL(k))/2)*sin(thetaVett(k));
            thetaVett(k+1)= thetaVett(k)+ ((uR(k)-uL(k))/d);
%                 k
        end   
    else
        uR(k) = (deltaRho(k) + (d/2)*deltaTheta(k));
        uL(k) = (deltaRho(k) - (d/2)*deltaTheta(k));

        xVett(k+1)= xVett(k)+ ((uR(k)+uL(k))/2)*cos(thetaVett(k));
        yVett(k+1)= yVett(k)+ ((uR(k)+uL(k))/2)*sin(thetaVett(k));
        thetaVett(k+1)= thetaVett(k)+ ((uR(k)-uL(k))/d);
%             k
    end
    k = k + 1;        
%         pause
end

% Passaggio dagli spostamenti deltaRho e deltaTheta agli spostamenti delle
% due ruote
uR = (deltaRho + (d/2)*deltaTheta)/deltaRvera;
uL = (deltaRho - (d/2)*deltaTheta)/deltaLvera;

x0 = xVett(1);
y0 = yVett(1);    
theta0 = thetaVett(1);    
    
% Caratteristiche dinamiche robot
NiR = randn(nPassi,1)*sqrt(KR); % sulle letture odometriche di traslazione
NiL = randn(nPassi,1)*sqrt(KL); % sulle letture odometriche di traslazione

% Letture odometriche corrotte
uRe = zeros(nPassi,1); % vettore letture odometriche ruota destra
uLe = zeros(nPassi,1); % vettore letture odometriche ruota sinistra
for k = 1:nPassi
    uRe(k) = uR(k) + sqrt(abs(uR(k)))*NiR(k); %encoders reading - letture odometriche ruota destra
    uLe(k) = uL(k) + sqrt(abs(uL(k)))*NiL(k); %encoders reading - letture odometriche ruota sinistra
end