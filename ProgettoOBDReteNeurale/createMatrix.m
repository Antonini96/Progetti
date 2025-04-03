function [X, Y] = createMatrix(filename)

    % Apertura file in lettura
    fid = fopen(filename, 'r');
    
    % Inizializzazione di due matrice vuote per contenere i dati
    X = [];
    
    Y = [];
    
    % si legge ogni riga del file e estrai i valori dopo i ':'
    while ~feof(fid)
        % lettura una riga dal file
        line = fgetl(fid);
    
        %solo il primo valore di riga, cioe l'etichetta
        label = textscan(line, '%f %*d:%*f %*d:%*f %*d:%*f %*d:%*f %*d:%*f %*d:%*f %*d:%*f %*d:%*f', 'Delimiter', ' ', 'CollectOutput', true);
    
        %aggiungo i valori alla matrice numerica
        Y = [Y; label{1}];
        
        % divisione della riga utilizzando ':' come delimitatore e conversione dei valori dopo i ':' in numeri
        values = textscan(line, '%*f %*d:%f %*d:%f %*d:%f %*d:%f %*d:%f %*d:%f %*d:%f %*d:%f', 'Delimiter', ' ', 'CollectOutput', true);
        
        % Aggiungo i valori alla matrice numerica
        X = [X; values{1}];
    end
    
    % Chiusura del file
    fclose(fid);

end

