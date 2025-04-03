%funzione che carica i dati e li separa in featurs e labels
function [x_train, y_train, x_test, y_test, vettore_labels] = divideMatrix()

    %caso binario - unisco i file associati a questo dataset
    filename1 = 'codrnaXTrain.mat';
    filename2 = 'codrnaYTrain.mat';
    X = load(filename1);
    Y = load(filename2);
    x = X.X;
    y = Y.Y;
    filename1 = 'codrnaXTest.mat';
    filename2 = 'codrnaYTest.mat';
    Xtest = load(filename1);
    Ytest = load(filename2);
    x = vertcat(x, Xtest.X);
    y = vertcat(y, Ytest.Y);
    filename1 = 'codrnaunused.mat';
    Xun = load(filename1);
    xun = Xun.codrnaunused1;
    x = vertcat(x, xun(:, 2:end));
    y = vertcat(y, xun(:, 1));

    % standardizzazione, tipo di normalizzazione
    mean_X = mean(x);
    std_X = std(x);
    for i = 1:length(std_X)
        if std_X(i) == 0
            std_X(i) = 1;
        end
    end
    X_normalized = (x - mean_X) ./ std_X;

    %allocazione statica per velocizzare il processo rispetto a quella dinamica
    xt = zeros(size(x, 1), size(x, 2));
    yt = zeros(length(y), 1);
    
    %ordino in maniera randomica x e y
    indexs = randsample(length(y), length(y));
    for i=1:length(y)
        xt(i, :) = X_normalized(indexs(i), :);
        yt(i, :) = y(indexs(i));
    end

    vettore_labels = transpose(unique(yt)); %vettore contenente in ordine le labels

    %separare il set in due parti, trainingset e testset
    h = size(xt, 1); %h=n campioni
    
    perc_testset = 0.25; %percentuale della dimensione del test set
    
    x_test = xt(end-int64(h*perc_testset):end,:); %matrice x del test set
    y_test = yt(end-int64(h*perc_testset):end, 1); %matrice y del test set
    
    xtrain = xt(1:end-int64(h*perc_testset)-1, :); %matrice x del training set
    ytrain = yt(1:end-int64(h*perc_testset)-1, :); %matrice y del training set

    %traspongo e metto per colonne le features e le labels
    x_train = transpose(xtrain);
    y_train = transpose(ytrain);
    x_test = transpose(x_test);
    y_test = transpose(y_test);
end

