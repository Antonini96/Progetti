%algoritmo che inizia con il LMKF e poi continua con il EKF

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

% Matrice covarianza posa robot (2x2)
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

%variabili per lo switch degli algoritmi
passoCambio = 1; % 1 uso LMKF, 0 uso EKF
cambio = 1; % inizializzo EKF per la prima volta, 0 non lo inizializzo più
val_minimo = sqrt((xVett(1)-xHat(1)).^2+(yVett(1)-yHat(1)).^2);
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%      INIZIO         CICLO    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t0 = clock;

err = 0;
for i = 1:nTag

    err = err + sqrt((cTag(i, 1)-Xr(2*i+1, 1)).^2+(cTag(i, 2)-Xr(2*i+2, 1)).^2); %salvo l'errore tra le posizioni stimati dei landamark e quelli veri

end
cTag_transMKF(1) = err/nTag;

while k < nPassi

    if passoCambio %Inizio LMKF

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

        % Stima dell'orientamento basata sulla stima di due posizioni
        % consecutive
        thetaHat(k) = atan2(yHat(k+1)-yHat(k),xHat(k+1)-xHat(k));

        %switch filtro
        errore = sqrt((xVett(k+1)-xHat(k+1)).^2+(yVett(k+1)-yHat(k+1)).^2);

        if errore - val_minimo > 1 %se il valore cresce di più di un centimetro
            passoCambio = 0; %switcho al EKF
        elseif errore < val_minimo %se il nuovo valore e minore del valore minimo
            val_minimo = errore; %diventa valore minimo
        end

        err = 0;
        for i = 1:nTag
        
            err = err + sqrt((cTag(i, 1)-Xr(2*i+1, k+1)).^2+(cTag(i, 2)-Xr(2*i+2, k+1)).^2); %salvo l'errore tra le posizioni stimati dei landamark e quelli veri
        
        end
        cTag_transMKF(k+1) = err/nTag;

    else %inizio EKF

        if cambio

            %inizializzazione EKF
            % Per semplicita' scrivo la P in questo modo molto approssimato
            P = blkdiag(P, sigmaBussola^2); %matrice di covarianza (bussola in ultima posizione)
            idx = [1:2, 2*nTag+3, 3:2*nTag+2]; %ordine desiderato
            P = P(idx, idx); %riaggiorno la P mettendo la bussola in posizione 3

            F = eye(2*nTag+3, 2*nTag+3);
            W = zeros(2*nTag+3, 2);
            W(3, 1) = 1/d;
            W(3, 2) = -1/d;
            H = zeros(2*nTag+1, 2*nTag+3); %da cambiare le prime 2 colonne
            H(1, 3) = 1; %dovuta dalla misurazione della bussola
            Li = eye(2*nTag+1);
            R = eye(2*nTag+1); %matrice di covarianza del rumore di z
            R(1, 1) = sigmaBussola^2; %nella prima posizione varianza della bussola
            for indTag = 2:2:2*nTag+1
                R(indTag, indTag) = sigmaRange^2;
                R(indTag+1, indTag+1) = sigmaBearing^2;
            end

            %aumento lo stato esteso
            Xr = [Xr(1:2, :); thetaHat'; Xr(3:end, :)];

            kVal = k + 1; %per curiosità mi salvo il valore di k dove switcho

            cambio = 0;

        end

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
        cTag_transMKF(k+1) = err/nTag;

    end

    k = k + 1;

end

%posizione dei landmark stimati
cTagHat_EKFmisto = zeros(nTag, 2);
for indTag = 1:nTag
    cTagHat_EKFmisto(indTag, 1) = Xr(2*indTag+2, end); %coordinara x dell'indTag landmark
    cTagHat_EKFmisto(indTag, 2) = Xr(2*indTag+3, end); %coordinara x dell'indTag landmark
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