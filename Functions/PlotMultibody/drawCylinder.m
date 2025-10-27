function drawCylinder(ax,T_start, T_end, diameter_cyl, color)
% Disegna un cilindro da T_start a T_end, lungo la direzione Z di T_start.
% Non richiede toolbox esterni (no axang2rotm).
%
% INPUT:
%   - T_start: [4x4] sistema di riferimento di partenza
%   - T_end: [4x4] sistema di riferimento di arrivo (stessa direzione Z)
%   - diameter_cyl: diametro del cilindro
%   - color: colore del cilindro (es. 'r', [0.5 0.5 0.5], ecc.)

% Author: Antonio Filianoti

    % Estrai posizioni
    p_start = T_start(1:3, 4);
    p_end   = T_end(1:3, 4);

    % Calcola direzione e lunghezza
    direction = p_end - p_start;
    length_cyl = norm(direction);

    if length_cyl == 0
        warning('Lunghezza cilindro nulla. Nessun cilindro disegnato.');
        return;
    end

    % Crea cilindro standard lungo Z
    [X, Y, Z] = cylinder(diameter_cyl / 2, 30);
    Z = Z * length_cyl;

    % Punti cilindro in 3xN
    pts = [X(:), Y(:), Z(:)]';

    % Direzione target e asse Z di riferimento
    z_target = direction / length_cyl;
    z_ref = [0; 0; 1];

    % Verifica se rotazione è necessaria
    if norm(cross(z_ref, z_target)) < 1e-6
        R_align = eye(3); % Nessuna rotazione necessaria
    else
        % Calcola matrice di rotazione con formula di Rodrigues
        v = cross(z_ref, z_target);
        s = norm(v);
        c = dot(z_ref, z_target);
        vx = [    0   -v(3)   v(2);
               v(3)     0   -v(1);
              -v(2)  v(1)     0];
        R_align = eye(3) + vx + vx^2 * ((1 - c) / (s^2));
    end

    % Applica rotazione e traslazione
    pts_transf = R_align * pts + p_start;

    % Ricostruisci mesh trasformata
    X_t = reshape(pts_transf(1,:), size(X));
    Y_t = reshape(pts_transf(2,:), size(Y));
    Z_t = reshape(pts_transf(3,:), size(Z));

    % Disegna cilindro
    surf(ax,X_t, Y_t, Z_t, 'FaceColor', color, 'EdgeColor', 'none');
    fill3(ax,X_t(1,:), Y_t(1,:), Z_t(1,:), color, 'EdgeColor', 'none');
    fill3(ax,X_t(2,:), Y_t(2,:), Z_t(2,:), color, 'EdgeColor', 'none');


end


