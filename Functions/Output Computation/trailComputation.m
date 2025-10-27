function [trail, NormalTrail, p_intersect, Trail_vec] = trailComputation(T_out, P, Output)
% COMPUTE_TRAIL  - Calcola l'avancorsa (trail) dato T_out e P da MultibodyMatrices
%
% Sintassi:
%   [trail, p_intersect, Trail_vec] = compute_trail(T_out, P)
%
% Descrizione:
%   Questa funzione calcola la distanza (trail) sul piano strada tra:
%     - il punto di contatto anteriore P.p_contatto_ant
%     - l'intersezione dell'asse di sterzo con il piano strada
%
%   Input:
%     T_out : struct restituito da MultibodyMatrices (usa i campi T_SDR_abs e T_steering_axis)
%     P     : struct con i punti chiave (usa P.p_contatto_ant)
%
%   Output:
%     trail       : valore scalare della avancorsa (metri se input in metri)
%     p_intersect : [3x1] punto di intersezione asse-terrno (coordinate globali)
%     Trail_vec   : [3x1] vettore 3D da P.p_contatto_ant a p_intersect
%
% Note:
%   - La funzione assume che il piano strada sia definito da T_out.T_SDR_abs.
%   - L'asse di sterzo è fornito come retta passante per T_out.T_steering_axis(1:3,4)
%     e direzione T_out.T_steering_axis(1:3,3) (colonna z locale).
%   - Non fa controlli di input (usa direttamente i campi come presenti in T_out e P).

% estraggo piano strada (punto e normale)
T_SDR = T_out.T_SDR_abs;
p_plane = T_SDR(1:3,4);        % punto su piano strada
n_plane = T_SDR(1:3,3);        % normale al piano strada (asse Z di SDR)

% punto e direzione dell'asse di sterzo
T_steer = T_out.T_steering_axis;
p_axis = T_steer(1:3,4);       % punto sull'asse di sterzo (in globale)
d_axis = T_steer(1:3,3);       % direzione dell'asse di sterzo (colonna z locale)

% calcolo parametro t per l'intersezione retta (p_axis + t*d_axis) con il piano:
% condizione: n_plane' * (p_axis + t*d_axis - p_plane) = 0
t = ( n_plane' * (p_plane - p_axis) ) / ( n_plane' * d_axis );

% punto di intersezione
p_intersect = p_axis + t * d_axis;

% vettore trail e valore scalare (proiezione sul piano X-Z)
% estraggo solo componenti X e Z per la distanza orizzontale sul suolo
dx = p_intersect(1) - P.p_contatto_ant(1);
dz = p_intersect(3) - P.p_contatto_ant(3);
trail = sqrt( dx.^2 + dz.^2 );

% vettore 3D completo (utile per plotting)
Trail_vec = p_intersect - P.p_contatto_ant;




    % Trail normale = distanza tra i due punti lungo x_axis
        NormalTrail = trail*cosd(Output.caster_angle);
end

