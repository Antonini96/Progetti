%comparazione tra i filtri EKF, LMKF e EKFmisto
clc
clear 
close all

nProve = 100;
nPassi = 3000;

sigmaBussola = 15*pi/180;

nTag = 2;

slam = 0; %fare lo SLAM 1; non fare lo SLAM 0
save = 0; %1 salvare i grafici; 0 altrimenti

kVett = zeros(nProve, 1); %mi salvo i valori di k dove switcho filtro

set(0, 'DefaultFigurePosition', [50, 50, 1000, 600]);

dati;

Jnormale = zeros(nProve,1);
Jlineare = zeros(nProve,1);
Jmisto = zeros(nProve, 1);

iter_est = zeros(nPassi, nProve);
iter_int = zeros(nPassi, nProve);
iter_misto = zeros(nPassi, nProve);

err_final = zeros(nProve, 3);


%salvataggio stime landmark
land_hatEKF = zeros(nPassi, nProve);
land_hatLMKF = zeros(nPassi, nProve);
land_hatMKF = zeros(nPassi, nProve);



for indProva = 1:nProve
    
    indProva
    
    percorsoRandom;

    if slam
        EKF_SLAM
        land_hatEKF(:, indProva) = cTag_transEKF;
    else
        % EKFnormale
        EKF
    end
    Jnormale(indProva) = J;

    iter_est(:, indProva) = errore;
    err_final(indProva, 1) = errore(end);

    if slam
        LMKF_SLAM
        land_hatLMKF(:, indProva) = cTag_transLMKF;
    else
        % EKFlineare
        LMKF
    end
    Jlineare(indProva) = J;

    iter_int(:, indProva) = errore;
    err_final(indProva, 2) = errore(end);

    if slam
        EKFmisto_SLAM
        land_hatMKF(:, indProva) = cTag_transMKF;
    else
        EKFmisto
    end
    Jmisto(indProva) = J;

    iter_misto(:, indProva) = errore;
    err_final(indProva, 3) = errore(end);

    kVett(indProva) = kVal;

end

iter_est_mean = mean(iter_est, 2); %media per righe degli errori su ogni passo
iter_int_mean = mean(iter_int, 2);
iter_misto_mean = mean(iter_misto, 2);

% %% -----------------------------------------------------------------------
% % plot professore degli errori medi
% figure
% plot(sort(Jnormale)','r.--', 'MarkerSize', 20, 'LineWidth', 2)
% hold on
% plot(sort(Jlineare),'k.--', 'MarkerSize', 20, 'LineWidth', 2)
% plot(sort(Jmisto), 'g.--', 'MarkerSize', 20, 'LineWidth', 2)
% xlabel('Numero di passi', 'FontSize', 14);
% ylabel('RMSE [cm]', 'FontSize', 14);
% legend('EKF','LMKF', 'EKFmisto')
% grid on
% title({'RMSE medio', ['\sigma_b = ', num2str(rad2deg(sigmaBussola)), '; Landmark = ', num2str(nTag)]}, 'FontSize', 16);
% 
% if save
%     if slam
%         %salvo il grafico del RMSE finale
%         filename = sprintf('C:\\Users\\3204480343\\OneDrive\\Desktop\\Esami da fare\\Tesi\\Foto Tesi\\RMSEmedio_SLAM_%dL%dS.svg', nTag, rad2deg(sigmaBussola));
%         saveas(gcf, filename);
%     else
%         %salvo il grafico del RMSE finale
%         filename = sprintf('C:\\Users\\3204480343\\OneDrive\\Desktop\\Esami da fare\\Tesi\\Foto Tesi\\RMSEmedio_%dL%dS.svg', nTag, rad2deg(sigmaBussola));
%         saveas(gcf, filename);
%     end
% end
% 
% %% -----------------------------------------------------------------------
%visualizzazione degli errori con valore finale e nel transitorio
%transitorio

figure;
plot(0:1:nPassi-1, iter_est_mean, 'r-', 'MarkerSize', 16, 'LineWidth', 3);
hold on
plot(0:1:nPassi-1, iter_int_mean, 'k-', 'MarkerSize', 16, 'LineWidth', 3);
plot(0:1:nPassi-1, iter_misto_mean, 'g-', 'MarkerSize', 16, 'LineWidth', 3);

%aumento la dimensione degli assi
ax = gca;  % Ottieni l'oggetto degli assi
ax.FontSize = 16;  % Cambia il numero con la dimensione desiderata
yscale log %ti fa l'asse delle y in scala logaritmica
% Legenda e etichette
legend('Transitorio EKF', 'Transitorio LMFK', 'Transitorio EKFmisto', 'FontSize', 12);
xlabel('Numero di passi', 'FontSize', 14);
ylabel('Errore assoluto medio [cm]', 'FontSize', 14);
title({'Errore assoluto sul transitorio', ['\sigma_b = ', num2str(rad2deg(sigmaBussola)), '; Landmark = ', num2str(nTag)]}, 'FontSize', 16);
grid on;

if save
    if slam
        %salvo il grafico del RMSE transitorio
        filename = sprintf('C:\\Users\\3204480343\\OneDrive\\Desktop\\Esami da fare\\Tesi\\Foto Tesi\\EAtransitorio_SLAM_%dL%dS.svg', nTag, rad2deg(sigmaBussola));
        saveas(gcf, filename);
    else
        %salvo il grafico del RMSE transitorio
        filename = sprintf('C:\\Users\\3204480343\\OneDrive\\Desktop\\Esami da fare\\Tesi\\Foto Tesi\\EAtransitorio_%dL%dS.svg', nTag, rad2deg(sigmaBussola));
        saveas(gcf, filename);
    end
end

% %% -------------------------------------------------------------------------%
% %valore finale
% err_final_sort = sort(err_final, 1); %ordino in ordine crescente le colonne
% figure;
% plot(err_final_sort(:, 1), 'ro', 'MarkerSize', 16, 'LineWidth', 2);
% hold on
% plot(err_final_sort(:, 2), 'ko', 'MarkerSize', 16, 'LineWidth', 2);
% plot(err_final_sort(:, 3), 'go', 'MarkerSize', 16, 'LineWidth', 2);
% 
% ax = gca;  % Ottieni l'oggetto degli assi
% ax.FontSize = 16;  % Cambia il numero con la dimensione desiderata
% 
% % Legenda e etichette
% legend('e_a finale EKF', 'e_a finale finale LMFK', 'e_a finale EKFmisto','FontSize', 12);
% xlabel('Numero di simulazioni', 'FontSize', 14);
% ylabel('Errore assoluto [cm]', 'FontSize', 14);
% title({'Errore assoluto finale ', ['\sigma_b = ', num2str(rad2deg(sigmaBussola)), '; Landmark = ', num2str(nTag)]}, 'FontSize', 16);
% grid on;
% 
% if save %salvare i grafici
%     if slam
%         %salvo il grafico del RMSE finale
%         filename = sprintf('C:\\Users\\3204480343\\OneDrive\\Desktop\\Esami da fare\\Tesi\\Foto Tesi\\EAfinale_SLAM_%dL%dS.svg', nTag, rad2deg(sigmaBussola));
%         saveas(gcf, filename);
%     else
%         %salvo il grafico del RMSE finale
%         filename = sprintf('C:\\Users\\3204480343\\OneDrive\\Desktop\\Esami da fare\\Tesi\\Foto Tesi\\EAfinale_%dL%dS.svg', nTag, rad2deg(sigmaBussola));
%         saveas(gcf, filename);
%     end
% end


%% ------------------------------------------------------------------------
%Errore landmark
if slam

    land_hatEKF_mean = mean(land_hatEKF, 2); %media per righe degli errori su ogni passo
    land_hatLMKF_mean = mean(land_hatLMKF, 2);
    land_hatMKF_mean = mean(land_hatMKF, 2);

    figure;
    plot(0:1:nPassi-1, land_hatEKF_mean, 'r-', 'MarkerSize', 16, 'LineWidth', 3);
    hold on
    plot(0:1:nPassi-1, land_hatLMKF_mean, 'k-', 'MarkerSize', 16, 'LineWidth', 3);
    plot(0:1:nPassi-1, land_hatMKF_mean, 'g-', 'MarkerSize', 16, 'LineWidth', 3);

    %aumento la dimensione degli assi
    ax = gca;  % Ottieni l'oggetto degli assi
    ax.FontSize = 16;  % Cambia il numero con la dimensione desiderata
    % yscale log %ti fa l'asse delle y in scala logaritmica
    % Legenda e etichette
    legend('Landmark EKF', 'Landmark LMFK', 'Landmark EKFmisto', 'FontSize', 12);
    xlabel('Numero di passi', 'FontSize', 14);
    ylabel('Errore assoluto medio [cm]', 'FontSize', 14);
    title({'Errore assoluto sul transitorio', ['\sigma_b = ', num2str(rad2deg(sigmaBussola)), '; Landmark = ', num2str(nTag)]}, 'FontSize', 16);
    grid on;

    %salvo il grafico del RMSE finale
    % filename = sprintf('C:\\Users\\3204480343\\OneDrive\\Desktop\\Esami da fare\\Tesi\\Foto Tesi\\landmarkStimati\\Misto\\land_trans_%dL%dS.svg', nTag, rad2deg(sigmaBussola));
    % saveas(gcf, filename);

end