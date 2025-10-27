function drawFrame(T_in, label, scale)
% drawFrame - Disegna un sistema di riferimento 3D usando quiver3
%
% Input:
%   T_in  : [4x4] matrice omogenea del frame da disegnare
%   label : stringa con il nome da usare per le etichette degli assi
%   scale : (opzionale) lunghezza degli assi (default = 0.1)

% Author: Antonio Filianoti

    if nargin < 2 || isempty(label)
        label = '';  % Default label vuota
    end
    if nargin < 3
        scale = 0.1;  % Default scale
    end

    % Estrai l'origine e gli assi dal sistema di riferimento T_in
    origin = T_in(1:3, 4);
    x_axis = T_in(1:3, 1) * scale;
    y_axis = T_in(1:3, 2) * scale;
    z_axis = T_in(1:3, 3) * scale;

    % Disegna gli assi con quiver3 (ΔX, ΔY, ΔZ sono i vettori direzionali)
    quiver3(origin(1), origin(2), origin(3), x_axis(1), x_axis(2), x_axis(3), ...
        'Color', 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(origin(1), origin(2), origin(3), y_axis(1), y_axis(2), y_axis(3), ...
        'Color', 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    quiver3(origin(1), origin(2), origin(3), z_axis(1), z_axis(2), z_axis(3), ...
        'Color', 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5);

    % Etichette (posizionate correttamente alla fine degli assi)
    text(origin(1) + x_axis(1), origin(2) + x_axis(2), origin(3) + x_axis(3), ...
    ['X_{' label '}'], 'Color', 'r', 'FontSize', 10, 'HorizontalAlignment', 'center');

    text(origin(1) + y_axis(1), origin(2) + y_axis(2), origin(3) + y_axis(3), ...
    ['Y_{' label '}'], 'Color', 'g', 'FontSize', 10, 'HorizontalAlignment', 'center');

    text(origin(1) + z_axis(1), origin(2) + z_axis(2), origin(3) + z_axis(3), ...
    ['Z_{' label '}'], 'Color', 'b', 'FontSize', 10, 'HorizontalAlignment', 'center');

end



