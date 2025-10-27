function mu_opt = findMuForFlatFrontContact(T_centro_post, T_SDR_abs, P_local, data,P)
%FINDMUFORFLATFRONTCONTACT Calcola l'angolo di beccheggio (mu) che porta
% la ruota anteriore a toccare tangenzialmente il piano stradale senza penetrazione.
%
% INPUT:
%   T_centro_post : [4x4] Matrice di trasformazione del centro ruota posteriore
%   T_SDR_abs     : [4x4] Matrice di trasformazione assoluta del piano stradale (sistema SDR)
%   P_local       : [3xN] Punti del toroide anteriore nel sistema locale ruota
%   data          : Struttura contenente tutti i parametri cinematici e geometrici del sistema
%
% OUTPUT:
%   mu_opt        : Angolo ottimale di beccheggio (in radianti) per contatto tangente della ruota anteriore

% AUTHOR:
%   Antonio Filianoti

    %% === ESTRAZIONE PARAMETRI ===

    AlphaSwingArm       = data.Multibody.AlphaSwingArm;
    gamma               = data.Multibody.Gamma;  %% gamma regolazione dovuta a piastre
    SteeringAngle       = data.Multibody.SteeringAngle;

    LengthSwingArmFinal = data.Multibody.LengthSwingArmFinal;
    BetaSDRFrame           = data.Multibody.BetaSDRFrame;
    LengthFrameFinal    = data.Multibody.LengthFrameFinal;


    PlatesForkOffset    = data.FrontSuspension.PlatesForkOffset;
    ForksWheelbase      = data.FrontSuspension.ForksWheelbase;
    ForksVerticalOffset = data.FrontSuspension.ForksVerticalOffset;
    ForksLength         = data.FrontSuspension.ForksLength;
    ForksCompression    = data.FrontSuspension.Compression;
    FootOffset          = data.FrontSuspension.FootOffset;

    % Operatori di rototraslazione
    [~, Ry, Rz, T_rot, T_tr] = getOperators();

    %% === Piano stradale ===
    T_inv_plane  = inv(T_SDR_abs);          % Trasformazione inversa del piano
    
    %% === Funzione obiettivo: penetrazione rispetto al piano ===
    function minZ = penetrationDepth(mu)
        % Calcola la minima quota Z del toroide anteriore rispetto al piano stradale
        % con un angolo di beccheggio mu.

        % === KINEMATICA ANTERIORE ===
        T_frame_pivot = T_centro_post ...
                        * T_rot(Ry(mu)) ...
                        * T_rot(Ry(AlphaSwingArm)) ...
                        * T_tr([LengthSwingArmFinal; 0; 0]) ...
                        * T_rot(Ry(-BetaSDRFrame));

        T_steering_axis = T_frame_pivot ...
                        * T_tr([P.Coordinate2D.l_frame; 0; P.Coordinate2D.h_frame]) ...
                        * T_rot(Ry(-gamma)) ...
                        * T_rot(Rz(SteeringAngle));

        p_steering = T_steering_axis(1:3,4);
        R_steering = T_steering_axis(1:3,1:3);

        % === FORCELLA SUPERIORE DX e SX ===
        v_offset_dx = [PlatesForkOffset; -ForksWheelbase/2; ForksVerticalOffset];
        v_offset_sx = [PlatesForkOffset;  ForksWheelbase/2; ForksVerticalOffset];

        p_sup_dx = p_steering + R_steering * v_offset_dx;
        p_sup_sx = p_steering + R_steering * v_offset_sx;

        % === ESTREMITÀ INFERIORE FORCELLA ===
        v_fork_end = -(ForksLength - ForksCompression);
        p_end_dx = p_sup_dx + R_steering(:,3) * v_fork_end;
        p_end_sx = p_sup_sx + R_steering(:,3) * v_fork_end;

        % === PIEDINI FORCELLA ===
        p_foot_dx = p_end_dx + R_steering(:,1) * FootOffset;
        p_foot_sx = p_end_sx + R_steering(:,1) * FootOffset;

        % === CENTRO RUOTA ANTERIORE ===
        lateral_delta = dot(p_foot_sx - p_foot_dx, R_steering(:,2));
        p_centro_ant  = p_foot_dx + 0.5 * lateral_delta * R_steering(:,2);
        T_centro_ant  = [R_steering, p_centro_ant; 0 0 0 1];

        % === PUNTI DEL TOROIDE NELLO SPAZIO DEL PIANO ===
        P_global = T_centro_ant * P_local;
        P_plane  = T_inv_plane * P_global;

        minZ = min(P_plane(3,:)); % minima quota (Z) rispetto al piano
    end

    %% === VINCOLO DI TANGENZA: minZ == 0 ===
    function [c, ceq] = tangencyConstraint(mu)
        c   = [];               % Nessun vincolo di disuguaglianza
        ceq = penetrationDepth(mu);  % Condizione di contatto tangente
    end

    %% === COSTO: penalizza penetrazione sotto il piano ===
    costFunc = @(mu) max(0, -penetrationDepth(mu)); % Solo penetrazione è penalizzata

    %% === OTTIMIZZAZIONE ===
    mu0 = 0;                   % stima iniziale
    bounds = 1e3;              % limiti larghi
    opts = optimoptions('fmincon','Display','none');
    mu_opt = fmincon(costFunc, mu0, [], [], [], [], -bounds, bounds, @tangencyConstraint, opts);
end
