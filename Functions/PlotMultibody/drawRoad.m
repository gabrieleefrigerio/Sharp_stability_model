function drawRoad(ax,T_plane, range)
    % Author: Antonio Filianoti

    % drawRoad  Disegna un piano rettangolare inclinato nello spazio 3D
    %
    %   drawRoad(T_plane, range)
    %
    % INPUT:
    %   T_plane — trasformazione omogenea 4×4 del piano
    %   range   — estensione del piano in coordinate locali
    %   ax -asse su cui plottare

    % Creo una griglia 2×2 in XY
    [X, Y] = meshgrid(linspace(-range,  2*range, 2), ...
                      linspace(-range/2, range/2, 2));
    Z = zeros(size(X));  % piano z=0 in coordinate locali

    % Mesh di punti omogenei
    P_local  = [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];

    % Trasformo in globale
    P_global = T_plane * P_local;

    % Riformatto per il plot
    Xr = reshape(P_global(1,:), size(X));
    Yr = reshape(P_global(2,:), size(Y));
    Zr = reshape(P_global(3,:), size(Z));

    % Disegno il piano
    surf(ax,Xr, Yr, Zr, ...
         'FaceColor', [0.2 0.2 0.2], ...
         'EdgeColor', 'none', ...
         'FaceAlpha', 0.5);
end