function [mu_opt] = findMuForFlatFrontContact(T_centro_post, T_SDR_abs, data, P, P_local_front)
% REBUILD + ROOTFIND: applica rotazione mu al centro posteriore,
% ricostruisce la moto fino al centro ruota anteriore + toroide e misura
% la penetrazione del toroide rispetto al piano (signed min distance).
%
% INPUT:
%   T_centro_post    : 4x4 trasformazione del centro ruota post iniziale
%   T_SDR_abs        : 4x4 trasformazione del piano strada assoluto
%   data, P          : struct con i parametri (come in MultibodyMatrices)
%   P_local_front    : 3xN o 4xN mesh del toroide anteriore (local coords)
%
% OUTPUT:
%   mu_opt           : angolo di beccheggio (rad) che minimizza/azera la penetrazione
%
% Note: usa lo stesso ordine di trasformazioni di MultibodyMatrices.
%
% Autore: Antonio Filianoti 

    %% Preliminari
    % operatori (come nel tuo codice principale)
    [Rx, Ry, Rz, T_rot, T_tr] = getOperators();

    % assicurati P_local_front in omogenee 4xN
    if size(P_local_front,1) == 3
        P_local_front = [P_local_front; ones(1,size(P_local_front,2))];
    end

    % costanti piano
    n  = T_SDR_abs(1:3,3);   % normale piano
    p0 = T_SDR_abs(1:3,4);   % punto sul piano

    %% Funzione che, per un dato mu, ricostruisce la moto e ritorna la
    % distanza minima (signed) del toroide anteriore rispetto al piano.
    function dmin = penetration(mu)
        % Applica la rotazione di beccheggio AL CENTRO POSTERIORE (right-multiply
        % come nel tuo codice originale: T_centro_post = T_centro_post * T_rot(Ry(mu)))
        T_post_mu = T_centro_post * T_rot(Ry(mu));

        % === FORCELONE POST (ricostruzione come in MultibodyMatrices) ===
        T_mid = T_post_mu * T_rot(Ry(data.Multibody.AlphaSwingArm));
        T_swing_pivot = T_mid * T_tr([data.Multibody.LengthSwingArmFinal; 0; 0]);

        % === TELAIO PIGNONE E ASSE DI STERZO ===
        T_frame_pivot = T_swing_pivot * T_rot(Ry(-data.Multibody.AlphaSwingArm));
        T_frame_pivot = T_frame_pivot * T_rot(Ry(-data.Multibody.BetaSDRFrame));

        % Traslazione pignone (ma non è necessario per la front)
        % v_pinion_local = [data.Frame.PinionX; data.Frame.PinionY; data.Frame.PinionZ];
        % T_pinion = T_frame_pivot * T_tr(v_pinion_local);

        % === ASSE DI STERZO ===
        T_steering_axis = T_frame_pivot * T_tr([P.Coordinate2D.l_frame; 0; P.Coordinate2D.h_frame]);
        T_steering_axis = T_steering_axis * T_rot(Ry(-data.Multibody.Gamma)) * T_rot(Rz(data.Multibody.SteeringAngle));

        % --- PIATRE / PLATES FORK (come nel main) ---
        v_dx = [data.FrontSuspension.PlatesForkOffset + data.FrontSuspension.OffsetTop; -data.FrontSuspension.ForksWheelbase/2; data.FrontSuspension.PlatesTopThickness];
        v_sx = [data.FrontSuspension.PlatesForkOffset + data.FrontSuspension.OffsetTop;  data.FrontSuspension.ForksWheelbase/2; data.FrontSuspension.PlatesTopThickness];

        T_piastra_sup_dx = T_steering_axis * T_tr(v_dx);
        T_piastra_sup_sx = T_steering_axis * T_tr(v_sx);

        % --- CORPO FORCELLA (come nel main) ---
        T_sup_forc_dx = T_piastra_sup_dx * T_tr([0;0;data.FrontSuspension.ForksVerticalOffset]);
        T_sup_forc_sx = T_piastra_sup_sx * T_tr([0;0;data.FrontSuspension.ForksVerticalOffset]);

        length_1 = (7/10) * (data.FrontSuspension.ForksLength);
        T_mid_forc_dx = T_sup_forc_dx * T_tr([0;0; -length_1]);
        T_mid_forc_sx = T_sup_forc_sx * T_tr([0;0; -length_1]);

        length_2 = (3/10)*(data.FrontSuspension.ForksLength) - data.FrontSuspension.Compression;
        T_end_forc_dx = T_mid_forc_dx * T_tr([0;0; -length_2]);
        T_end_forc_sx = T_mid_forc_sx * T_tr([0;0; -length_2]);

        % --- PIEDINI FORCELLA ---
        T_foot_dx = T_end_forc_dx * T_tr([data.FrontSuspension.FootOffset; 0; 0]);
        T_foot_sx = T_end_forc_sx * T_tr([data.FrontSuspension.FootOffset; 0; 0]);

        p_foot_dx = T_foot_dx(1:3,4);
        p_foot_sx = T_foot_sx(1:3,4);

        % === CENTRO RUOTA ANTERIORE ===
        delta = dot(p_foot_sx - p_foot_dx, T_foot_dx(1:3,2));
        p_centro_ant = p_foot_dx + 0.5 * delta * T_foot_dx(1:3,2);

        T_centro_ant_loc = T_foot_dx;
        T_centro_ant_loc(1:3,4) = p_centro_ant;

        % --- Trasforma il toroide frontale al mondo
        P_global_front = T_centro_ant_loc * P_local_front; % 4xN

        % --- distanza signed dei punti dal piano: n'*(P - p0)
        zvals = n' * (P_global_front(1:3,:) - p0); % 1xN

        % distanza minima (se negativa => penetrato)
        dmin = min(zvals);

        % salva qualche risultato opzionale per debug (non globale)
        % (ma non sovrascriviamo variabili esterne)
    end

    mu_opt = fsolve(@(mu) abs(penetration(mu)), 0,  optimset('Display','off'));

    % %% === Campionamento e ricerca radice ===
    % maxdeg = 20;
    % mu_lo = -maxdeg * pi/180;
    % mu_hi =  maxdeg * pi/180;
    % 
    % Nsample = 41;
    % mus = linspace(mu_lo, mu_hi, Nsample);
    % vals = zeros(size(mus));
    % 
    % % fprintf('--- Campionamento penetrazione su [%g,%g] deg ---\n', mu_lo*180/pi, mu_hi*180/pi);
    % % for k = 1:numel(mus)
    % %     vals(k) = penetration(mus(k));
    % %     fprintf(' mu = %+6.3f deg  -> dmin = %+8.6g m\n', mus(k)*180/pi, vals(k));
    % % end
    % 
    % % cerca bracket
    % idx_br = find(vals(1:end-1).*vals(2:end) < 0, 1);
    % if ~isempty(idx_br)
    %     a = mus(idx_br); b = mus(idx_br+1);
    %     % fprintf('Bracket trovato tra %g° e %g°\n', a*180/pi, b*180/pi);
    %     try
    %         mu_opt = fzero(@penetration, [a b], optimset('TolX',1e-1,'Display','off'));
    %     catch err
    %         %warning('fzero fallito: %s. Uso fallback fminbnd.', err.message);
    %         mu_opt = fminbnd(@(x) abs(penetration(x)), mu_lo, mu_hi, optimset('TolX',1e-1,'Display','off'));
    %     end
    % else
    %     %warning('Nessun bracket trovato, uso fallback fminbnd su abs(dmin).');
    %     mu_opt = fminbnd(@(x) abs(penetration(x)), mu_lo, mu_hi, optimset('TolX',1e-1,'Display','off'));
    % end
    % 
    % % ultimo sanity
    % if ~isfinite(mu_opt)
    %     %warning('mu_opt non finito: imposto 0');
    %     mu_opt = 0;
    % end

    % %% === Ricostruzione finale con mu_opt per ritornare trasformazioni e punto contatto ===
    % T_centro_post_new = T_centro_post * T_rot(Ry(mu_opt));
    % 
    % % ricostruisco esattamente come nella funzione penetration per ottenere T_centro_ant e p_contatto
    % T_mid = T_centro_post_new * T_rot(Ry(data.Multibody.AlphaSwingArm));
    % T_swing_pivot = T_mid * T_tr([data.Multibody.LengthSwingArmFinal; 0; 0]);
    % 
    % T_frame_pivot = T_swing_pivot * T_rot(Ry(-data.Multibody.AlphaSwingArm));
    % T_frame_pivot = T_frame_pivot * T_rot(Ry(-data.Multibody.BetaSDRFrame));
    % 
    % T_steering_axis = T_frame_pivot * T_tr([P.Coordinate2D.l_frame; 0; P.Coordinate2D.h_frame]);
    % T_steering_axis = T_steering_axis * T_rot(Ry(-data.Multibody.Gamma)) * T_rot(Rz(data.Multibody.SteeringAngle));
    % 
    % v_dx = [data.FrontSuspension.PlatesForkOffset + data.FrontSuspension.OffsetTop; -data.FrontSuspension.ForksWheelbase/2; data.FrontSuspension.PlatesTopThickness];
    % v_sx = [data.FrontSuspension.PlatesForkOffset + data.FrontSuspension.OffsetTop;  data.FrontSuspension.ForksWheelbase/2; data.FrontSuspension.PlatesTopThickness];
    % 
    % T_piastra_sup_dx = T_steering_axis * T_tr(v_dx);
    % T_piastra_sup_sx = T_steering_axis * T_tr(v_sx);
    % 
    % T_sup_forc_dx = T_piastra_sup_dx * T_tr([0;0;data.FrontSuspension.ForksVerticalOffset]);
    % T_sup_forc_sx = T_piastra_sup_sx * T_tr([0;0;data.FrontSuspension.ForksVerticalOffset]);
    % 
    % length_1 = (7/10) * (data.FrontSuspension.ForksLength);
    % T_mid_forc_dx = T_sup_forc_dx * T_tr([0;0; -length_1]);
    % T_mid_forc_sx = T_sup_forc_sx * T_tr([0;0; -length_1]);
    % 
    % length_2 = (3/10)*(data.FrontSuspension.ForksLength) - data.FrontSuspension.Compression;
    % T_end_forc_dx = T_mid_forc_dx * T_tr([0;0; -length_2]);
    % T_end_forc_sx = T_mid_forc_sx * T_tr([0;0; -length_2]);
    % 
    % T_foot_dx = T_end_forc_dx * T_tr([data.FrontSuspension.FootOffset; 0; 0]);
    % T_foot_sx = T_end_forc_sx * T_tr([data.FrontSuspension.FootOffset; 0; 0]);
    % 
    % p_foot_dx = T_foot_dx(1:3,4);
    % p_foot_sx = T_foot_sx(1:3,4);
    % 
    % delta = dot(p_foot_sx - p_foot_dx, T_foot_dx(1:3,2));
    % p_centro_ant = p_foot_dx + 0.5 * delta * T_foot_dx(1:3,2);
    % 
    % T_centro_ant = T_foot_dx;
    % T_centro_ant(1:3,4) = p_centro_ant;
    % 
    % % trasforma la mesh frontale
    % P_global_front_final = T_centro_ant * P_local_front;
    % zvals_final = n' * (P_global_front_final(1:3,:) - p0);
    % [dmin_final, idx_min_final] = min(zvals_final);
    % p_contatto_ant = P_global_front_final(1:3, idx_min_final);
    % 
    % %% === Print di debug finale ===
    % fprintf('\n=== RISULTATO FINALE ===\n');
    % fprintf('mu_opt = %+8.5f rad = %+7.3f deg\n', mu_opt, mu_opt*180/pi);
    % fprintf('dmin_final = %+12.9f m  (min signed distance torus-plane)\n', dmin_final);
    % fprintf('p_centro_ant = [%.6f, %.6f, %.6f]\n', p_centro_ant(1), p_centro_ant(2), p_centro_ant(3));
    % fprintf('p_contatto_ant = [%.6f, %.6f, %.6f]\n', p_contatto_ant(1), p_contatto_ant(2), p_contatto_ant(3));
    % fprintf('T_centro_post_new translation = [%.6f, %.6f, %.6f]\n', T_centro_post_new(1:3,4));
    % 
    % %% === Debug struct e plot ===
    % debug.mus = mus;
    % debug.pen_vals = vals;
    % debug.mu_opt = mu_opt;
    % debug.dmin_final = dmin_final;
    % debug.p_centro_ant = p_centro_ant;
    % debug.p_contatto_ant = p_contatto_ant;
    % 
    % % plot
    % figure; plot(mus*180/pi, vals, '-o','LineWidth',1.2);
    % hold on; plot(mu_opt*180/pi, penetration(mu_opt), 'rs','MarkerFaceColor','r','MarkerSize',8);
    % xlabel('\mu [deg]'); ylabel('d_{min} (signed) [m]');
    % grid on; yline(0,'--k'); title('Penetrazione toroide anteriore vs \mu');
    % hold off;

end
