% plot([0 L/100],[0 0],'k','Linewidth',2)
% hold on
% axis([-1*(L/100)/100 (L/100)+1*(L/100)/100 -1*(L/100)/100 (L/100)+1*(L/100)/100])
axis equal
plot(xVett/100,yVett/100);%,':','Linewidth',1);
hold on
plot(cTag(:, 1)/100, cTag(:, 2)/100, 'rH', 'Linewidth',2);
% for indTag = 1:nTag,
%     plot(cTag(indTag,1)/100,cTag(indTag,2)/100,'rH','Linewidth',2);
% end

if slam
    plot(cTagHat_EKF(:, 1)/100, cTagHat_EKF(:, 2)/100, 'bo', 'Linewidth',2);
    plot(cTagHat_LMKF(:, 1)/100, cTagHat_LMKF(:, 2)/100, 'ko', 'Linewidth',2);
    plot(cTagHat_EKFmisto(:, 1)/100, cTagHat_EKFmisto(:, 2)/100, 'go', 'Linewidth',2);
    % for indTag = 1:nTag
    %     plot(cTagHat(indTag,1)/100,cTagHat(indTag,2)/100,'ko','Linewidth',2);
    % end
end

plot([0 L/100],[0 0],'k','Linewidth',2)
hold on
axis([-1*(L/100)/100 (L/100)+1*(L/100)/100 -1*(L/100)/100 (L/100)+1*(L/100)/100])
axis equal
plot([0 (L/100)],[(L/100) (L/100)],'k','Linewidth',2)
plot([0 0],[0 (L/100)],'k','Linewidth',2)
plot([(L/100) (L/100)],[0 (L/100)],'k','Linewidth',2)
plot(xVett(1)/100, yVett(1)/100, 'go', 'LineWidth', 6);

if slam
    legend('percorso uniciclo', 'landmark veri', 'landmark stimati EKF\_SLAM', 'landmark stimati LMKF\_SLAM', 'landmark stimati EKFmisto\_SLAM', 'Bordi ambiente');
else
    legend('percorso uniciclo', 'landmark', 'Bordi ambiente');
end

xlabel('posizione asse x [m]', 'FontSize', 14);
ylabel('posizione asse y [m]', 'FontSize', 14);
title('Ambiente');