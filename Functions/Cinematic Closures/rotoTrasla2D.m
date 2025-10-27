function p_global = rotoTrasla2D(orig, theta, rel)
%ROTOTRASLA2D Esegue una rototraslazione 2D
%
% Input:
%   - orig: [x0; z0] coordinate dell'origine locale (vettore colonna 2x1)
%   - angle_deg: angolo in radianti (rotazione antioraria rispetto all'asse x globale)
%   - rel: [x_rel; z_rel] coordinate relative nella terna locale (vettore colonna 2x1)
%
% Output:
%   - p_global: coordinate nel sistema globale (vettore colonna 2x1)

% Author: Antonio Filianoti


    % Matrice omogenea di rototraslazione 2D (3x3)
    T = [cos(theta), -sin(theta), orig(1);
         sin(theta),  cos(theta), orig(2);
         0,           0,          1];

    % Punto relativo in coordinate omogenee
    p_rel_h = [rel; 1];

    % Coordinate globali
    p_global_h = T * p_rel_h;

    % Ritorna solo x,z
    p_global = p_global_h(1:2);
end

