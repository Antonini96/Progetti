%algoritmo di EKF
gradi = pi/180;
% dati;
% percorsoRandom; 

% Stima posa robot
xHat = zeros(nPassi,1);
yHat = zeros(nPassi,1);
thetaHat = zeros(nPassi,1);

%variabili del filtro di kalman con bussola esterna

%variabili del filtro di kalman con bussola esterna
F = eye(3); %da cambiare F(0,2) e F(1,2)
P = [10 0 0; 0 10 0; 0 0 2]; %matrice di covarianza della posa
W = [0 0; 0 0; 1/d -1/d]; %da cambiare i valori in zero
Q = zeros(2); %da cambiare i termini nella diagonale
H = zeros(2*nTag+1, 3); %da cambiare le prime 2 colonne
H(1, 3) = 1; %dovuta dalla misurazione della bussola
Li = eye(2*nTag+1);
R = eye(2*nTag+1); %matrice di covarianza del rumore di z
R(1, 1) = sigmaBussola^2; %nella prima posizione varianza della bussola
for indTag = 2:2:2*nTag+1
    R(indTag, indTag) = sigmaRange^2;
    R(indTag+1, indTag+1) = sigmaBearing^2;
end

%parametri dell'innovazione (bussola + Landmark)
h = zeros(2*nTag+1, 1);
z = zeros(2*nTag+1, 1);
inn = zeros(2*nTag+1, 1);

k = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%      INIZIO         CICLO    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t0 = clock;

while k < nPassi

    %bussola vera (quindi sara dato dagli encoder rumorosi)
    bussola = thetaVett(k) + sigmaBussola*randn;

    %passo di predizione
    xHatMeno = xHat(k) + (uRe(k)+uLe(k))/2*cos(thetaHat(k));
    yHatMeno = yHat(k) + (uRe(k)+uLe(k))/2*sin(thetaHat(k));
    thetaHatMeno = thetaHat(k) + (uRe(k)-uLe(k))/d;

    %bussola stimata
    bussolaAttesa = thetaHatMeno;

    %variabili da usare al tempo k nel passo di predizione
    F(1, 3) = -(uRe(k)+uLe(k))/2*sin(thetaHat(k));
    F(2, 3) = (uRe(k)+uLe(k))/2*cos(thetaHat(k));
    W(1, 1) = 0.5*cos(thetaHat(k));
    W(1, 2) = 0.5*cos(thetaHat(k));
    W(2, 1) = 0.5*sin(thetaHat(k));
    W(2, 2) = 0.5*sin(thetaHat(k));
    Q(1, 1) = KR*abs(uRe(k));
    Q(2, 2) = KL*abs(uLe(k));

    Pmeno = (F*P*transpose(F)) + (W*Q*transpose(W));

    %misura bussola stimata
    h(1) = bussolaAttesa; 
        
    %misura bussola vera
    z(1) = bussola;

    for indTag=1:nTag %landmark

        xli = cTag(indTag,1); %posizione x dell'i-esimo landmark
        yli = cTag(indTag,2); %posizione y dell'i-esimo landmark

        x_cap = xHatMeno; %posizione x stimata dell'uniciclo
        y_cap = yHatMeno; %posizione y stimata dell'uniciclo

        %distanza del robot dai landmark con Xrmeno (senza theta)
        h(2*indTag) = sqrt((x_cap - xli)^2 + (y_cap - yli)^2);

        %distanza del robot dai landmark con rumore (senza theta)
        z(2*indTag) = sqrt((xVett(k+1) - xli)^2 + (yVett(k+1) - yli)^2) + sigmaRange*randn;

        %distanza del landmark dalla stima della posa senza rumore
        D = h(2*indTag);

        %angolo di bearing stimato
        h(2*indTag+1) = atan2(yli - y_cap, xli - x_cap) - thetaHatMeno;
    
        %angolo di bearing vero
        z(2*indTag+1) = atan2((yli - yVett(k+1)), (xli - xVett(k+1))) - thetaVett(k+1) + sigmaBearing*randn;

        %gradiente della h stato di osservabilità rispetto alla posa
        H(2*indTag, 1) = (x_cap - xli)/D;
        H(2*indTag, 2) = (y_cap - yli)/D;
    
        H(2*indTag+1, 1) = -(y_cap - yli)/((x_cap - xli)^2*((y_cap - yli)^2/(x_cap - xli)^2 + 1));
        H(2*indTag+1, 2) = 1/((x_cap - xli)*((y_cap - yli)^2/(x_cap - xli)^2 + 1));
        H(2*indTag+1, 3) = -1;

    end

    KalmanGain = Pmeno*transpose(H)*pinv(H*Pmeno*transpose(H) + Li*R*transpose(Li));

    inn = z - h;
    inn(1) = atan2(sin(inn(1)), cos(inn(1)));

    for indTag = 2:2:2*nTag+1
        inn(indTag+1) = atan2(sin(inn(indTag+1)), cos(inn(indTag+1)));
    end

    xHat(k+1) = xHatMeno + KalmanGain(1,:)*inn;
    yHat(k+1) = yHatMeno + KalmanGain(2,:)*inn;
    thetaHat(k+1) = thetaHatMeno + KalmanGain(3,:)*inn;

    P = (eye(3)-(KalmanGain*H))*Pmeno;

    k = k + 1;

end

DeltaT = etime(clock,t0);

errore = sqrt((xVett-xHat).^2+(yVett-yHat).^2); %errore ogni passo

J = mean(errore);

if DISEGNA
    figure
    disegnaFig
    plot(xHat/100,yHat/100,'c.');
    figure
    plot(errore)
end