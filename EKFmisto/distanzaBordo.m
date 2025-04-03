function [lato, distanza] = distanzaBordo(Xt,Yt,Thetat)

tol = .001;
RaggioCirconfRobot = 0.01;

numVertici = 4;

P=zeros(numVertici,2); % matrice coordinate vertici
P(1,:)=[0,0];
P(2,:)=[400,0];
P(3,:)=[400,400];
P(4,:)=[0,400];

% M = [P1 P2; P2 P3; P3 P4; ... P_{numVertici} P1];  % matrice dei lati: la
% riga i contiene i due vertici del lato i del perimetro, quindi ci sono 
% tante righe quanti sono i lati del perimetro esterno dell'ambiente
M = zeros(numVertici,4);
for indVertice = 1:numVertici-1
    M(indVertice,:) = [P(indVertice,:) P(indVertice+1,:)];
end
M(numVertici,:) = [P(numVertici,:) P(1,:)];

numSegmenti = numVertici; % numero dei lati del perimetro dell'ambiente

% Coefficienti delle rette contenenti i lati del perimetro
n = ones (1,numSegmenti); % coefficienti angolari rette
q = ones (1,numSegmenti); % intercette rette
for lato = 1:numSegmenti
   P1x = M(lato,1);
   P1y = M(lato,2);
   P2x = M(lato,3);   
   P2y = M(lato,4);   
   if P1x==P2x
       n(lato) = Inf;
       q(lato) = P1x;  % in questo caso il coefficiente q ricorda la x dell'equazione x = q
   else
       n(lato) = (P2y-P1y)/(P2x-P1x); % coefficiente angolare lato
       q(lato) = P1y - n(lato)*P1x; % costante lato
   end
end


% la lettura minima dal laser al perimetro (alla fine deve dare il valore ok)
letturaMin = Inf;  
latoMin = 0;

% coordinate del laser
Xi = Xt + RaggioCirconfRobot*cos(Thetat);
Yi = Yt + RaggioCirconfRobot*sin(Thetat);

% intersezione con i lati del raggio del laser considerato e Verifica
% che e' effettivamente una intersezione 
XpBordo = zeros(1,numSegmenti);
YpBordo = zeros(1,numSegmenti);

% Calcolo coordinate intersezione del raggio del laser con i vari lati
for lato = 1:numSegmenti
    if n(lato) < Inf  % lato non verticale
        if abs(tan(Thetat) - n(lato)) < 0.0001 % raggio laser parallelo a lato
              XpBordo(lato) = Inf;
              YpBordo(lato) = Inf;                  
        else
              XpBordo(lato) = (Xt*tan(Thetat)-Yt+q(lato))/(tan(Thetat)-n(lato));
              YpBordo(lato) = ((n(lato)*Xt+q(lato))*tan(Thetat)-n(lato)*Yt)/(tan(Thetat)-n(lato));
        end
    else     % lato verticale
        XpBordo(lato) = q(lato);
        if abs(cos(Thetat)) < 0.0001  % raggio laser parallelo a lato verticale
            YpBordo(lato) = Inf;
%                 disp('raggio laser parallelo a lato verticale');                
        else 
            YpBordo(lato) = Yt + (q(lato)-Xt)*tan(Thetat);                  
        end
    end

    % verifica che intersezione appartiene al lato e sta dalla parte giusta
    if (XpBordo(lato) >= min(M(lato,1),M(lato,3))-tol) && (XpBordo(lato) <= max(M(lato,1),M(lato,3))+tol) && (YpBordo(lato) >= min(M(lato,2),M(lato,4))-tol) && (YpBordo(lato) <= max(M(lato,2),M(lato,4))+tol),  % l'intersezione appartiene al lato
        if (Xi >= min(XpBordo(lato),Xt)-tol) && (Xi <= max(XpBordo(lato),Xt)+tol) && (Yi >= min(YpBordo(lato),Yt)-tol) && (Yi <= max(YpBordo(lato),Yt)+tol), % il lato sta dalla parte giusta
            distLato =  sqrt((Xt-XpBordo(lato))^2+(Yt-YpBordo(lato))^2);   
            if distLato < letturaMin
                letturaMin = distLato;
                latoMin = lato;
            end
        end
    end

end

% si aggiorna con la distanza minima trovata sia la distanza che il lato corrispondente
distanza = letturaMin;
lato = latoMin;