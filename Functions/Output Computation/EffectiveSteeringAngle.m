function angle_deg = EffectiveSteeringAngle(T)
% EffectiveSteeringAngle Calcola l'angolo di sterzata effettivo in gradi
%
% INPUT:
%   T : struttura che contiene i seguenti campi:
%       - T.T_contatto_post : matrice 4x4 del sistema solidale con il punto di contatto posteriore
%       - T.T_centro_ant    : matrice 4x4 del sistema solidale con il centro della ruota anteriore
%
% OUTPUT:
%   angle_deg : angolo di sterzata effettivo (in gradi), positivo verso sinistra
%
% DESCRIZIONE:
%   L'angolo di sterzata è calcolato come l'angolo tra:
%   - la direzione longitudinale del veicolo (asse X posteriore)
%   - la direzione della ruota anteriore (asse X anteriore)
%   entrambi proiettati sul piano stradale (XY)
%
% AUTHOR:
%   Antonio Filianoti

    % Estrai la direzione X dal sistema posteriore (direzione del moto)
    x_moto = T.T_contatto_post(1:3, 1);
    
    % Estrai la direzione X dal sistema anteriore (orientamento ruota)
    x_ruota = T.T_centro_ant(1:3, 1);

    % Proietta entrambi i vettori sul piano XY (azzera la Z)
    x_moto(3) = 0;
    x_ruota(3) = 0;

    % Normalizza i vettori
    x_moto = x_moto / norm(x_moto);
    x_ruota = x_ruota / norm(x_ruota);

    % Calcola angolo in radianti usando formula atan2 tra due vettori
    angle_rad = atan2(norm(cross(x_moto, x_ruota)), dot(x_moto, x_ruota));

    % Conversione in gradi
    angle_deg = rad2deg(angle_rad);
end
