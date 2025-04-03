%lo sparo si inizia a vedere all'indice 39/40
%per ignorare un valore o not usare ~

%ad ogni sparo diverso cambiare nome file e riga 138 inserire il A succ.

close all
clear all
clc

filename = 'KLDT-O5WB-99971-1.mat';

fprintf('Sparo: %s\n\n', filename);

%mi mostra l'array tvec e le specifiche del video, scritte qui sopra
V = importdata(filename); %funziona solo con video.mat
disp(V); %mi fa vedere l'array di tvec, con tutti i tempi dei frames e
         %l'array delle specifiche di V (frames, width, height, matrix rgb)
%%

sizeVideo = size(V.Video); %ritorna le dimensioni di ogni campo di V.Video (matrix 1x4)

%number of frames
numFrames = sizeVideo(1, 1);
fprintf('number of frames: ');
disp(numFrames);

%valori iniziali nel caso non ci fosse il marfe
marfe = 0; 
tdec = 0.0000;

%valori iniziale del old_cy, prima che si trovi uno valido
old_cy = 1; %primo pixels, cosi impossibile che il valore sia >=20

stabile = 800.0000; %soglia per vedere se la old_cy va bene
good_stabile = 1; %per modificare solo una volta il old_cy

%per ogni frame
for index=1:numFrames

    tic; %inizio il conteggio del tempo

    Frame = V.Video(index, :, :, :); %unico frame
    %in pratica tolgo index e prendo solo gli altri parametri [2 3 4]
    Frame = reshape(Frame, sizeVideo([2, 3, 4])); %estrapolazione frame

    %trasforma il frame a colori rgb in scala di grigio
    frame_gray = rgb2gray(Frame);

    %Calculate a threshold. The threshold is normalized to the range [0, 1]
    T = graythresh(frame_gray);

    %somma del valore dei pixel identificati dalla threshold
    pixels_int = frame_gray >= T;  %trovo i valori dei pixel identificati dalla threshold
    intensity(index, 1) = sum(frame_gray(pixels_int));  %li sommo (per la media mean)


    %trasformo l'immagine in binary, dove piu intensa=1, meno intensa=0
    binary_image = imbinarize(frame_gray, T);

    %baricentro/centroide
    onexcolum = sum(binary_image, 1); %sommo gli 1 delle colonne
    centrx = onexcolum*[1:size(binary_image, 2)]'/sum(onexcolum); %calcolo centroide x
    
    onexrow = sum(binary_image, 2); %stessa cosa per le righe, centroide y
    centry = onexrow'*[1:size(binary_image, 1)]'/sum(onexrow);


    %allocazione dinamica di una matrice per le posizioni del centroide
    Mcentroide(index, 1) = centrx; 
    Mcentroide(index, 2) = centry;


    if good_stabile == 1 %entro finche non trovo valori validi

        if stabile <= centry %prendo i valori del centroide del primo valido

            fprintf('\n\n\nindice di riferimento: ');
            disp(index);
            
            %valori di riferimento del centroide
            old_cx=centrx;
            old_cy=centry;

            fprintf('old_cx, old_cy: %.4f %.4f\n\n\n', old_cx, old_cy);

            figure;

            imshow(binary_image); %frame binary di riferimento

            hold on;
            plot(centrx, centry,'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'g');

            good_stabile = 0; %non entro piu nel ciclo if
         
        end

    end


    %controllo se il nuovo centroide supera una certa soglia
    if old_cy-centry>=20 %se il centroide supera una certa soglia marfe=1

        marfe = 1; %avviene il marfe

        fprintf('\n\n\n\nindice di detenzione: ');
        disp(index);

        tdec = V.tvec(index); %tempo di detenzione
        fprintf('tempo di detenzione: ');
        disp(tdec);

        fprintf('centrx, centry: %.4f %.4f\n', centrx, centry);

        fprintf('\nMarfe trovato\n');

        figure;
        imshow(binary_image); %ultima immagine dove viene superata questa soglia

        hold on;
        plot(centrx, centry,'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'g');

        break; %interrompo il ciclo for
      
    end %end second if

    toc %fine conteggio tempo e stampa valore sulla command windows

end %end for

if marfe == 0

    fprintf('\nMarfe non trovato\n');

end

%scrittura dei risultati in un file excel
% file_excel = 'DatiMarfe.xlsx';
% 
% values = {filename, marfe, tdec};
% 
% xlswrite(file_excel, values, 1, 'A2');