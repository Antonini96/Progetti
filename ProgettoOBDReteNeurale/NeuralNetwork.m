%classe per la rete neurale
classdef (Sealed) NeuralNetwork

    %attributi rete neurale
    properties (SetAccess = private) %privati (non modificabili nel main)
        
    end
    properties %pubblici (modificabili nel main)
        %parametri
        Features %neuroni nello strato di input
        Classes %neuroni nello strato di output
        Layer_Neurons %neuroni di tutti i strati
        N_HiddenLayer %numero di strati totali
        epochs %epoche
        lr %learning rate
        vettore_labels %vettore contenente le etichette ordinate
        n_sample %numero di campioni da prendere per volta

        %pesi e bias
        W1 %peso strato input-primo hiddenlayer (hidden_lay(1)xfeatures)
        B1 %bias strato input-primo hiddenlayer (hidden_lay(1)x1)
        W2 %peso primo hiddenlayer-secondo hiddenlayer (hidden_lay(2)xhidden_lay(1))
        B2 %bias primo hiddenlayer-secondo hiddenlayer (hidden_lay(2)x1)
        W3 %peso secondo hiddenlayer-strato output (classesxhidden_lay(2))
        B3 %bias secondo hiddenlayer-strato output (classesx1)

        A1 % (hidden_lay(1)xn_sample)
        Z1 % (hidden_lay(1)xn_sample)
        A2 % (hidden_lay(2)xn_sample)
        Z2 % (hidden_lay(2)xn_sample)
        A3 % (classesxn_sample)
        Z3 % (classesxn_sample)

        outputArray %output Z3 dell'ultimo strato softmatizzato

        %funzione di perdita
        E

        %metriche
        acc_vector
        lossfunc
    end

    methods

        %inizializzazione dei pesi e reset degli output e delle metriche
        function obj = initializeWeights(obj)

            % Inizializzazione dei pesi e dei bias
            %peso e bias strato input-primo hiddenlayer
            obj.W1 = randn(obj.Layer_Neurons(2), obj.Layer_Neurons(1));
            obj.B1 = zeros(obj.Layer_Neurons(2), 1);
            %peso e bias primo hiddenlayer-secondo hiddenlayer
            obj.W2 = randn(obj.Layer_Neurons(3), obj.Layer_Neurons(2));
            obj.B2 = zeros(obj.Layer_Neurons(3), 1);
            %peso e bias secondo hiddenlayer-strato output
            obj.W3 = randn(obj.Layer_Neurons(4), obj.Layer_Neurons(3));
            obj.B3 = zeros(obj.Layer_Neurons(4), 1);

            obj.outputArray = []; %resetto l'output
            obj.acc_vector = []; %resetto il vettore delle accuracy
            obj.lossfunc = []; %resetto il vettore delle loss function

        end

        %Accuratezza della rete neurale per problemi di classificazione
        function acc = accuracy(obj, y, y_pred)

            %y_pred è l'ouputArray, dove, gia softmaxizzato, devo prendere
            %l'etichetta con probabilità maggiore
            a = y_pred;

            for i = 1:length(a)
                y_corrente = find(a(:, i) == max(a(:, i)), 1, "first");
                y_scelto(i) = obj.vettore_labels(y_corrente);
            end            

            acc = sum(y==y_scelto)./length(y); %confronto i risultati con l'etichetta vera
        end

        %funzione softmax
        function a = softmax_layer(~, n)
            a = exp(n)./(sum(exp(n)));
        end

        %funzione di perdita (1xcampioni) gli passo y(1xn_sample) e outputArray(2xn_sample)
        %Misura la differenza tra la y vera e la y predetta
        function E = logloss(obj, y)

            %y_true che gli passo è composta da -1 e 1, quindi la devo
            %adattare all'outputArray
            m = size(y, 2); % Numero di campioni

            %etichette vere
            y_true = zeros(obj.Classes, m);
        
            % Codifica binaria per le etichette -1[1, 0] e 1[0, 1]
            for i = 1:m 
                ind = find(obj.vettore_labels == y(i));
                y_true(ind, i) = 1;
            end

            %cross entropy
            sum1 = 0;
            for ii = 1:m
                sum2 = 0;
                for hh = 1:obj.Classes
                    expValue = exp(obj.outputArray(hh,ii));
                    sum2 = sum2 + expValue;
                end
                for jj = 1:obj.Classes
                    errorValue = y_true(jj,ii).*(log(exp(obj.outputArray(jj,ii))./sum2));
                    sum1 = sum1 + errorValue;
                end
            end
            E = -sum1./m; %Errore
        end

        %funzione di attivazione ReLu
        function ReLu = relu(~, Z)
            ReLu = max(Z, 0);
        end

        %funzione di attivazione sigmoid
        function Sigm = sigmoid(~, Z)
            Sigm = 1./(1+exp(-Z)); %funzione logistica
            %Sigm = (exp(Z)-exp(-Z))./(exp(Z)+exp(-Z)); %tangente iperbolica
        end

        %gradiente
        function gradient = relu_gradient(~, Z)
            gradient = double(Z > 0); %restituisce 1 se il valore è maggiore di 0, 0 altrimenti
        end

        %derivata della crossentropy rispetto alle y_predette
        function dEdy = derivate_label(obj, y_true, j, p)

            %calcolo la sommatoria delle etichette vere tranne quella corrente j
            a = 0;
            for i = 1:obj.Classes
                if i ~= j
                    a = a + y_true(i, p);
                end
            end

            %esponenziale di tutte le etichette segnate/predette
            supp = exp(obj.A3(j, p));

            %sommatoria di tutte le etichette segnate/predette esponenzializzate
            S = 0;
            for i = 1:obj.Classes
                S = S + exp(obj.A3(i, p));
            end

            dEdy = -(y_true(j, p)/S)*(S - supp);

            dEdy = dEdy + (supp/S)*a;

        end

        %forward propagation
        function obj = forward_propagation(obj, X, j, k)

            % Calcolo dell'output del primo strato nascosto
            obj.A1 = obj.W1*X - obj.B1;
            obj.Z1 = relu(obj, obj.A1);

            % Calcolo dell'output del secondo strato nascosto
            obj.A2 = obj.W2*obj.Z1 - obj.B2;
            obj.Z2 = relu(obj, obj.A2);

            % Calcolo dell'output finale
            obj.A3 = obj.W3*obj.Z2 - obj.B3;
            if obj.Classes == 2 %nel caso binario conviene usare la sigmoide
                obj.Z3 = sigmoid(obj, obj.A3);
            else %nel caso multiclasse conviene usare la relu
                obj.Z3 = relu(obj, obj.A3);
            end

            obj.Z3 = softmax_layer(obj, obj.Z3);
            obj.outputArray(:, j:j+k-1) = obj.Z3; %salvo gli output softmaxizzati
        end

        %addestramento
        function obj = back_propagation(obj, X, y, n_sample)
 
            dEdb1 = zeros(obj.Layer_Neurons(2),1);
            dEdb2 = zeros(obj.Layer_Neurons(3),1);
            dEdb3 = zeros(obj.Layer_Neurons(4),1);
            dEdw1 = zeros(obj.Layer_Neurons(2),obj.Features);
            dEdw2 = zeros(obj.Layer_Neurons(end-1),obj.Layer_Neurons(2));
            dEdw3 = zeros(obj.Classes,obj.Layer_Neurons(end-1));

            %calcolo y_true, cioe una matrice labels*numero di campioni dove è 1
            %se quel campione appartiene a quella etichetta, 0 altrimenti
            y_true = zeros(obj.Classes, n_sample); %labelsxn_sample
            for i=1:n_sample
                ind = find(obj.vettore_labels == y(i));
                y_true(ind, i) = 1;
            end

            %calcolo del ciclo for per n_sample campioni
            for p = 1:n_sample

                %per lo strato di output calcolo dE/dw
                for j = 1:obj.Classes %per tutte le classi
    
                    %calcolo dEda3(1xobj.Classes) = dE/dy_true*relu_gradient(a)    
                    dEdy(j) = derivate_label(obj, y_true, j, p);
    
                    dEda3(j) = dEdy(j).*relu_gradient(obj, obj.A3(j, p));
    
                    for i = 1:obj.Layer_Neurons(end-1) %per i neuroni del layer precedente all'output
    
                        %calcolo da/dw(obj.Classesxobj.LayerNeurons(end-1))
                        dadw(j,i) = obj.Z2(i,p);
    
                        %pongo dE/dw() = dE/da*da/dw 
                        dEdw3(j, i) = dEdw3(j, i) + dEda3(j).*dadw(j, i);
    
                    end
                    %Calcolo della derivata finale per lo strato di uscita
                    %rispetto alle componenti di bias (i = 0)
                    dEdb3(j,1) = dEdb3(j,1) - dEda3(j);
                end

                %per gli altri strati, cioe non L+1(output)
                for l = obj.N_HiddenLayer-1:-1:2 %per l che va da L,...,2 (il layer 0/input per me è il layer 1)
                    som = 0;
                    for j = 1:obj.Layer_Neurons(l)
                        
                        if l-1 == 2
                            %calcolo dEda2
                            for h = 1:obj.Layer_Neurons(l+1) %fino a Classes
                                som = som + dEda3(h).*obj.W3(h, j);
                            end 
                            for h = 1:obj.Layer_Neurons(l+1)
                                som = som + dEda3(h).*obj.B3(h);
                            end
                            dEda2(j) = relu_gradient(obj, obj.A2(j, p)).*som;
    
                            %calcolo dadw2
                            for i = 1:obj.Layer_Neurons(l-1)
                                dadw(j,i) = obj.Z1(i,p);
                                dEdw2(j, i) = dEdw2(j, i) + dEda2(j).*dadw(j, i);
                            end
                            dEdb2(j,1) = dEdb2(j,1) - dEda2(j);
                        end
                        
                        if l-1 == 1
                            %calcolo dEda1
                            for h = 1:obj.Layer_Neurons(l+1)
                                som = som + dEda2(h).*obj.W2(h, j);
                            end
                            for h = 1:obj.Layer_Neurons(l+1)
                                som = som + dEda2(h).*obj.B2(h);
                            end
                            dEda1(j) = relu_gradient(obj, obj.A1(j, p)).*som;
    
                            %calcolo dadw1
                            for i = 1:obj.Features
                                dadw(j,i) = X(i,p);
                                dEdw1(j, i) = dEdw1(j, i) + dEda1(j).*dadw(j, i);
                            end
                            dEdb1(j,1) = dEdb1(j,1) - dEda1(j);
                        end    
                    end
                end
            end

            % Aggiornamento dei pesi dopo lo studio dei tot campioni
            obj.W1 = obj.W1 - (obj.lr.*(dEdw1./n_sample));
            obj.W2 = obj.W2 - (obj.lr.*(dEdw2./n_sample));
            obj.W3 = obj.W3 - (obj.lr.*(dEdw3./n_sample));
            obj.B1 = obj.B1 - (obj.lr.*(dEdb1./n_sample));
            obj.B2 = obj.B2 - (obj.lr.*(dEdb2./n_sample));
            obj.B3 = obj.B3 - (obj.lr.*(dEdb3./n_sample));

        end

        %plot dei risultati finali (accuracy-LogLoss) per ogni epoca
        function plot_final(obj)
            
            %grafico dell'Accuracy totale
            subplot(2, 1, 1);
            plot(obj.acc_vector, 'b');
            xlabel('Iteration');
            ylabel('Accuracy (%)');
            title('Training Progress');

            %grafico della Loss
            subplot(2, 1, 2);
            plot(obj.lossfunc, 'r');
            xlabel('Iteration');
            ylabel('Loss');

        end

        %addestramento dei pesi
        function obj = fit(obj, X, Y)

            %inizializzazione dei pesi
            obj = initializeWeights(obj);
            
            contatore = 1; %per la waitbar

            h = waitbar(0, 'Avanzamento...'); % Crea una nuova barra di avanzamento

            maxCount = obj.epochs*size(X, 2); %tempo massimo da aspettare
            
            for i = 1:obj.epochs %epoca=insieme di iterazioni in cui abbiamo visitato tutto il training set

                tic %inizio a calcolare il tempo per ogni epoca

                fprintf("epoch number: ");
                disp(i);
                k = obj.n_sample;
                for j = 1:k:size(X, 2) %ne prendo k per volta 

                    if size(X, 2)-j<k %nel caso # campioni non sia k-multiplo
                        k = size(X, 2)-j+1;
                    end
                    x = X(:, j:j+k-1);
                    y = Y(j:j+k-1);
                    
                    %calcolo degli output
                    obj = forward_propagation(obj, x, j, k);

                    %calcolo la cross entropy (E) per ogni n_sample campioni di ogni epoca
                    obj.E(i, j:j+k-1) = obj.logloss(y);

                    %retropropagazione dell'errore, aggiornamento dei pesi
                    obj = back_propagation(obj, x, y, k); 

                    % Aggiorna la barra di avanzamento
                    waitbar(contatore / maxCount, h, sprintf('Avanzamento: %d%%', round(contatore / maxCount * 100)));

                    contatore = contatore+k; %aggiornamento del contatore/iterazioni totali
                end

                %salvo i valori finali dell'accuracy e della loss function per il plot
                obj.acc_vector(i) = accuracy(obj, Y, obj.outputArray);
                obj.lossfunc(i) = logloss(obj, Y);

                toc %fine calcolo del tempo
            end

            % Chiusura della barra di avanzamento
            close(h);

            plot_final(obj);

            AccuracyTrain = obj.acc_vector(end);

            fprintf("Accuracy del training: ");
            disp(AccuracyTrain);
        end

        %valutazione della rete neurale con il test set
        function obj = evaluate(obj, X, Y)

            %reinizializzo l'output, perche size differenti 
            obj.outputArray = [];

            %calcolo la y_predetta dei test
            obj = forward_propagation(obj, X, 1, size(X, 2)); %li considero tutti

            %calcolo l'accuratezza nei nuovi dati
            AccuracyTest = accuracy(obj, Y, obj.outputArray);

            fprintf("Accuracy del test: ");
            disp(AccuracyTest); 
        end

    end

end