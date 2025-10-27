function caster_angle = computeCasterAngle(T_steering_axis, T_SDR_abs)
    % computeCasterAngle - Calcola il caster angle rispetto a un SDR arbitrario
    %
    % INPUT:
    %   T_steering_axis - matrice 4x4 della trasformazione dell'asse sterzo
    %   T_SDR_abs       - matrice 4x4 del sistema di riferimento base
    %
    % OUTPUT:
    %   caster_angle    - angolo in gradi tra l'asse sterzo proiettato su XZ
    %                     e la verticale del SDR di riferimento
    %
    % Autore: Antonio Filianoti

    % Estrai l'asse Z locale dell'asse sterzo (direzione sterzo)
    z_local = T_steering_axis(1:3, 3);

    % Trasforma l'asse Z dello sterzo nel SDR di riferimento
    R_SDR = T_SDR_abs(1:3, 1:3);
    z_local_in_SDR = R_SDR' * z_local;

    % Proietta l'asse sul piano XZ del SDR
    z_proj = [z_local_in_SDR(1); 0; z_local_in_SDR(3)];
    z_proj = z_proj / norm(z_proj);  % Normalizzazione

    % Asse Z del SDR (cioè la verticale)
    z_SDR = [0; 0; 1];

    % Calcolo dell'angolo tra la proiezione e la verticale
    angle_rad = acos(dot(z_proj, z_SDR));
    caster_angle = rad2deg(angle_rad);  % Conversione in gradi
end



