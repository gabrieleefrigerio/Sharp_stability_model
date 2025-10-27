function [squat_angle, transfer_angle, squat_ratio, P] = compute_squat_ratio(P)
% COMPUTE_SQUAT_RATIO - Calcola l'angolo di squat, l'angolo di transfer
% e il rapporto R secondo il metodo di Cossalter.
%
% Tutti i punti utilizzati vengono presi da P.bikePlane (già proiettati).
%
% INPUT:
%   P    : struct con i punti della moto proiettati nel piano di simmetria
%          (accessibili come P.bikePlane.p_nome)
%   data : struct con parametri della trasmissione (non usato qui ma lasciato per compatibilità)
%
% OUTPUT:
%   squat_angle    : angolo della Squat Line rispetto all'asse X [gradi]
%   transfer_angle : angolo della Transfer Line [gradi]
%   squat_ratio    : rapporto R = tan(transfer_angle) / tan(squat_angle)
%   P              : struct aggiornato con il punto di intersezione

    %% --- Linea catena (tra punti superiori corona e pignone) ---
    m1 = (P.bikePlane.p_sup_pinion(3) - P.bikePlane.p_sup_sprocket(3)) / ...
         (P.bikePlane.p_sup_pinion(1) - P.bikePlane.p_sup_sprocket(1));
    q1 = P.bikePlane.p_sup_sprocket(3) - m1 * P.bikePlane.p_sup_sprocket(1);

    %% --- Linea forcellone (dal centro posteriore al pivot) ---
    m2 = (P.bikePlane.p_pivot(3) - P.bikePlane.p_centro_post(3)) / ...
         (P.bikePlane.p_pivot(1) - P.bikePlane.p_centro_post(1));
    q2 = P.bikePlane.p_centro_post(3) - m2 * P.bikePlane.p_centro_post(1);

    %% --- Intersezione tra linea catena e linea forcellone ---
    x_intersect = (q2 - q1) / (m1 - m2);
    z_intersect = m1 * x_intersect + q1;
    P.bikePlane.p_intersect = [x_intersect; 0; z_intersect];

    %% --- Angolo Squat (tra contatto posteriore e intersezione) ---
    squat_angle = atan2d(z_intersect - P.bikePlane.p_contatto_post(3), ...
                         x_intersect - P.bikePlane.p_contatto_post(1));

    %% --- Angolo Transfer (tra contatto posteriore e linea baricentro) ---
    x_transfer = P.bikePlane.p_contatto_ant(1);
    z_transfer = P.bikePlane.p_COG_tot(3);
    transfer_angle = atan2d(z_transfer - P.bikePlane.p_contatto_post(3), ...
                            x_transfer - P.bikePlane.p_contatto_post(1));

    %% --- Rapporto Squat ---
    squat_ratio = tand(transfer_angle) / tand(squat_angle);

end


%% --- DEBUG PLOT ---
% figure('Position', [100 100 1400 800]); hold on; grid on; axis equal;
% 
% % Linea 1
% x_line1 = linspace(p_sup_sprocket(1)-0.5, p_sup_pinion(1)+0.5, 200);
% z_line1 = p_sup_sprocket(3) + (p_sup_pinion(3)-p_sup_sprocket(3)) * ...
%           (x_line1 - p_sup_sprocket(1)) / (p_sup_pinion(1)-p_sup_sprocket(1));
% plot(x_line1, z_line1, '--b', 'LineWidth', 2, 'DisplayName', 'Linea 1');
% 
% % Linea 2
% x_line2 = linspace(p_centro_post(1)-0.5, p_pivot(1)+0.5, 200);
% z_line2 = m2 * x_line2 + q2;
% plot(x_line2, z_line2, '--g', 'LineWidth', 2, 'DisplayName', 'Linea 2');
% 
% % Intersezione
% scatter(x_intersect, z_intersect, 50, 'r', 'filled', 'DisplayName', 'Intersezione L1-L2');
% 
% % Squat Line
% plot([p_contatto_post(1), x_intersect], ...
%      [p_contatto_post(3), z_intersect], '-.m', 'LineWidth', 2, 'DisplayName', 'Squat Line');
% 
% % Transfer Line
% plot([p_contatto_post(1), x_transfer], ...
%      [p_contatto_post(3), z_transfer], '-.c', 'LineWidth', 2, 'DisplayName', 'Transfer Line');
% 
% % Punti chiave
% scatter([p_sup_pinion(1), p_sup_sprocket(1), p_contatto_post(1), p_centro_post(1)], ...
%         [p_sup_pinion(3), p_sup_sprocket(3), p_contatto_post(3), p_centro_post(3)], ...
%         50, 'k', 'filled');
% 
% %% --- Cerchi corona e pignone ---
% theta = linspace(0,2*pi,200);
% 
% r_sprocket = (data.Transmission.FinalSprocket * data.Transmission.FinalChainModule)/2 / pi;
% r_pinion   = (data.Transmission.FinalPinion   * data.Transmission.FinalChainModule)/2 / pi;
% 
% % Corona
% x_sprocket = p_centro_post(1) + r_sprocket*cos(theta);
% z_sprocket = p_centro_post(3) + r_sprocket*sin(theta);
% plot(x_sprocket, z_sprocket, 'k', 'LineWidth', 1.2, 'DisplayName','Corona');
% 
% % Pignone
% x_pinion = p_pinion(1) + r_pinion*cos(theta);
% z_pinion = p_pinion(3) + r_pinion*sin(theta);
% plot(x_pinion, z_pinion, 'k', 'LineWidth', 1.2, 'DisplayName','Pignone');
% 
% title('Debug Squat / Transfer Lines (frame moto)', 'FontSize', 14);
% xlabel('X_{moto} [m]'); ylabel('Z_{moto} [m]');
% legend('Location', 'Best'); grid on; axis equal;


