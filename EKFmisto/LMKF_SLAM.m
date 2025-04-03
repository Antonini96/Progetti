%algoritmo di LMKF_SLAM

% Definizione delle varie costanti e delle coordinate dei landmark
dati;

% Definzione del percorso del robot e degli errori odometrici
% percorsoRandom;

% Stima posa robot
xHat = zeros(nPassi,1);
yHat = zeros(nPassi,1);

Xr = zeros(2*nTag+2, nPassi); %stato esteso
Xr(1:2, 1) = [xHat(1); yHat(1)]; %posa (data dalla lettura degli encoder Ue)
for i = 1:nTag
    Xr(2*i+1, 1) = cTag(i, 1) + randn/10; %aggiungo il rumore sulla posizione x dell'iesimo landmark
    Xr(2*i+2, 1) = cTag(i, 2) + randn/10; %aggiungo il rumore sulla posizione  dell'iesimo landmark
end

%variabili del filtro di kalman con bussola interna
P = eye(2*nTag+2);
% Matrice covarianza posa robot (2x2)
P(1, 1) = 10;
P(2, 2) = 10;
for indTag = 1:nTag
    P(2*indTag+1, 2*indTag+1) = 1;
    P(2*indTag+2, 2*indTag+2) = 1;
end
% H = -eye(2); %matrice di kronecker
Lk = zeros(2, 2);
R = zeros(2);
ZitaAttesa = zeros(2, 1);
Zita = zeros(2, 1);
U = zeros(2, 1);
W = zeros(2);
Q = zeros(2);

R(1, 1) = sigmaRange^2;
R(2, 2) = sigmaBussola^2+sigmaBearing^2;

k = 1;
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%      INIZIO         CICLO    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t0 = clock;

err = 0;
for i = 1:nTag

    err = err + sqrt((cTag(i, 1)-Xr(2*i+1, 1)).^2+(cTag(i, 2)-Xr(2*i+2, 1)).^2); %salvo l'errore tra le posizioni stimati dei landamark e quelli veri

end
cTag_transLMKF(1) = err/nTag;

while k < nPassi

    % Misure della bussola e odometrica usate nel passo di predizione
    thetaBussola = thetaVett(k) + sigmaBussola*randn;
    uek = (uRe(k)+uLe(k))/2;

    %passo di predizione
    %variabili da usare al tempo k nel passo di predizione
    U(1) = uek*cos(thetaBussola);
    U(2) = uek*sin(thetaBussola);

    W(1, 1) = cos(thetaBussola);
    W(1, 2) = -uek*sin(thetaBussola);
    W(2, 1) = sin(thetaBussola);
    W(2, 2) = uek*cos(thetaBussola);

    Q(1, 1) = .25*(KR*abs(uRe(k))+KL*abs(uLe(k)));
    Q(2, 2) = sigmaBussola^2;

    xHatMeno = xHat(k) + U(1);
    yHatMeno = yHat(k) + U(2);

    Pmeno = P + blkdiag((W*Q*transpose(W)), zeros(2*nTag));

    %passo di correzione
    Ptemp = Pmeno;
    xTemp = xHatMeno;
    yTemp = yHatMeno;

    Xrmeno = [xTemp; yTemp; Xr(3:end, k)];

    %li uso solo per calcorare il bearing e per il range
    x_cap = xVett(k+1); %posizione vera x dell'uniciclo
    y_cap = yVett(k+1); %posizione vera y dell'uniciclo

    for indTag = 1:nTag

        H = MatriceDiKronecker(nTag, indTag);

        xli = cTag(indTag,1); %posizione x dell'i-esimo landmark
        yli = cTag(indTag,2); %posizione y dell'i-esimo landmark

        %angolo di bearing - questo me lo da la misura
        bearing = atan2(yli - y_cap, xli - x_cap) - thetaVett(k+1) + sigmaBearing*randn; %compresa di rumore
        %angolo totale - questo lo devo calcolare
        angoloTotMisurato = thetaBussola + bearing; %bussola calcolata da me - bearing dato dalla misura

        %compresa di rumore
        range = sqrt((x_cap - xli)^2 + (y_cap - yli)^2) + sigmaRange*randn; %distanza tra uniciclo e landmark

        Lk(1, 1) = cos(angoloTotMisurato);
        Lk(1, 2) = -range*sin(angoloTotMisurato);
        Lk(2, 1) = sin(angoloTotMisurato);
        Lk(2, 2) = range*cos(angoloTotMisurato);

        %misura attesa - Zita attesa
        ZitaAttesa(1) = xli - xHatMeno;
        ZitaAttesa(2) = yli - yHatMeno;

        %misura corretta
        Zita(1) = range*cos(angoloTotMisurato);
        Zita(2) = range*sin(angoloTotMisurato);

        %guadagno di Kalman
        KalmanGain = Ptemp*transpose(H)*pinv((H*Ptemp*transpose(H)) + (Lk*R*transpose(Lk)));

        innovation = Zita - ZitaAttesa;

        Xrmeno = Xrmeno + KalmanGain*innovation;

        % xTemp = xTemp + KalmanGain(1,:)*innovation;
        % yTemp = yTemp + KalmanGain(2,:)*innovation;

        Ptemp = (eye(2*nTag+2) - (KalmanGain*H))*Ptemp;

    end

    Xr(:, k+1) = Xrmeno;

    xHat(k+1) = Xr(1, k+1);
    yHat(k+1) = Xr(2, k+1);
    P = Ptemp;

    err = 0;
    for i = 1:nTag
    
        err = err + sqrt((cTag(i, 1)-Xr(2*i+1, k+1)).^2+(cTag(i, 2)-Xr(2*i+2, k+1)).^2); %salvo l'errore tra le posizioni stimati dei landamark e quelli veri
    
    end
    cTag_transLMKF(k+1) = err/nTag;

    k = k + 1;

end

%posizione dei landmark stimati
cTagHat_LMKF = zeros(nTag, 2);
for indTag = 1:nTag
    cTagHat_LMKF(indTag, 1) = Xr(2*indTag+1, end); %coordinara x dell'indTag landmark
    cTagHat_LMKF(indTag, 2) = Xr(2*indTag+2, end); %coordinara x dell'indTag landmark
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