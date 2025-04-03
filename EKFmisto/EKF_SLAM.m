%algoritmo di EKF_SLAM
gradi = pi/180;
% dati;
% percorsoRandom; 

% Stima posa robot
xHat = zeros(nPassi,1);
yHat = zeros(nPassi,1);
thetaHat = zeros(nPassi,1);

xHat(1) = xVett(1);
yHat(1) = yVett(1);
thetaHat(1) = thetaVett(1);

Xr = zeros(2*nTag+3, nPassi); %stato esteso
Xr(1:3, 1) = [xHat(1); yHat(1); thetaHat(1)];
% for i = 1:nTag
%     Xr(2*i+2, 1) = cTag(i, 1) + randn/10; %aggiungo il rumore sulla posizione x dell'iesimo landmark
%     Xr(2*i+3, 1) = cTag(i, 2) + randn/10; %aggiungo il rumore sulla posizione  dell'iesimo landmark
% end

%variabili del filtro di kalman con bussola esterna

%variabili del filtro di kalman con bussola esterna
F = eye(2*nTag+3, 2*nTag+3); %da cambiare F(0,2) e F(1,2)
P = zeros(2*nTag+3); %matrice di covarianza della posa
P(1, 1) = 0;
P(2, 2) = 0;
P(3, 3) = 0;
for i = 1:nTag
    P(2*i+2, 2*i+2) = 1000000000000;
    P(2*i+3, 2*i+3) = 1000000000000;
end
W = zeros(2*nTag+3, 2);
W(3, 1) = 1/d;
W(3, 2) = -1/d;
Q = zeros(2); %da cambiare i termini nella diagonale
H = zeros(2*nTag+1, 2*nTag+3); %da cambiare le prime 2 colonne
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

cTag_transEKF = zeros(1, nPassi);

err = 0;
for i = 1:nTag

    err = err + sqrt((cTag(i, 1)-Xr(2*i+2, 1)).^2+(cTag(i, 2)-Xr(2*i+3, 1)).^2); %salvo l'errore tra le posizioni stimati dei landamark e quelli veri

end
cTag_transEKF(1) = err/nTag;

cambio = 1;

while k < nPassi

    %bussola vera (quindi sara dato dagli encoder rumorosi)
    bussola = thetaVett(k+1) + sigmaBussola*randn;

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

        H(2*indTag, 2*indTag+2) = -H(2*indTag, 1);
        H(2*indTag, 2*indTag+3) = -H(2*indTag, 2);
    
        H(2*indTag+1, 1) = -(y_cap - yli)/((x_cap - xli)^2*((y_cap - yli)^2/(x_cap - xli)^2 + 1));
        H(2*indTag+1, 2) = 1/((x_cap - xli)*((y_cap - yli)^2/(x_cap - xli)^2 + 1));
        H(2*indTag+1, 3) = -1;

        H(2*indTag+1, 2*indTag+2) = -H(2*indTag+1, 1);
        H(2*indTag+1, 2*indTag+3) = -H(2*indTag+1, 2);

    end

    KalmanGain = Pmeno*transpose(H)*pinv(H*Pmeno*transpose(H) + Li*R*transpose(Li));

    inn = z - h;
    inn(1) = atan2(sin(inn(1)), cos(inn(1)));

    for indTag = 2:2:2*nTag+1
        inn(indTag+1) = atan2(sin(inn(indTag+1)), cos(inn(indTag+1)));
    end

    Xr(1:3, k+1) = [xHatMeno; yHatMeno; thetaHatMeno];

    Xr(:, k+1) = [xHatMeno; yHatMeno; thetaHatMeno; Xr(4:end, k)] + KalmanGain*inn;

    if cambio
        for i = 1:nTag
            Xr(2*i+2, k+1) = cTag(i, 1) + 10000*randn/10; %aggiungo il rumore sulla posizione x dell'iesimo landmark
            Xr(2*i+3, k+1) = cTag(i, 2) + 10000*randn/10; %aggiungo il rumore sulla posizione  dell'iesimo landmark
        end
        cambio = 0;
    end

    % xHat(k+1) = xHatMeno + KalmanGain(1,:)*inn;
    % yHat(k+1) = yHatMeno + KalmanGain(2,:)*inn;
    % thetaHat(k+1) = thetaHatMeno + KalmanGain(3,:)*inn;

    xHat(k+1) = Xr(1, k+1);
    yHat(k+1) = Xr(2, k+1);
    thetaHat(k+1) = Xr(3, k+1);

    P = (eye(2*nTag+3)-(KalmanGain*H))*Pmeno;

    err = 0;
    for i = 1:nTag
    
        err = err + sqrt((cTag(i, 1)-Xr(2*i+2, k+1)).^2+(cTag(i, 2)-Xr(2*i+3, k+1)).^2); %salvo l'errore tra le posizioni stimati dei landamark e quelli veri
    
    end
    cTag_transEKF(k+1) = err/nTag;

    k = k + 1;

end

%posizione dei landmark stimati
cTagHat_EKF = zeros(nTag, 2);
for indTag = 1:nTag
    cTagHat_EKF(indTag, 1) = Xr(2*indTag+2, end); %coordinara x dell'indTag landmark
    cTagHat_EKF(indTag, 2) = Xr(2*indTag+3, end); %coordinara x dell'indTag landmark
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