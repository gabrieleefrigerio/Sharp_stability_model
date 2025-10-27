function drawCOG(ax,position, radius, colorMoto, label)
    % drawCOG - Disegna una sfera con un pattern a scacchiera divisa in 4 quadranti,
    %           con colori: [coloreMoto, bianco].
    %
    % INPUT:
    %   position  - [3x1] coordinate del centro (X, Y, Z)
    %   radius    - raggio della sfera (default: 20)
    %   colorMoto - colore principale (char 'r','b','k', oppure [R G B])
    %   label     - (opzionale) stringa accanto al baricentro
    % 
    % AUTHOR:
    %   Antonio Filianoti


    
    if nargin < 3, radius = 20; end
    if nargin < 4, colorMoto = 'k'; end
    if nargin < 5, label = ''; end

    % Converte eventuale carattere ('r','b', ecc.) in RGB
    if ischar(colorMoto)
        colorMoto = colorSpecToRGB(colorMoto);
    end

    % Genera mesh sferica
    [X, Y, Z] = sphere(100);
    X = X * radius + position(1);
    Y = Y * radius + position(2);
    Z = Z * radius + position(3);

    % Conversione in coordinate sferiche
    [theta, phi, ~] = cart2sph(X - position(1), Y - position(2), Z - position(3));
    theta = mod(theta, 2*pi);

    % Pattern a scacchiera
    checker = mod(floor(2*phi/pi) + floor(2*theta/pi), 2);

    % Definizione colori
    color1 = colorMoto;       % colore moto
    color2 = [1, 1, 1];       % bianco

    % Costruzione matrice RGB
    C = zeros([size(checker), 3]);
    for i = 1:3
        C(:,:,i) = checker * color1(i) + (1 - checker) * color2(i);
    end

    % Disegno sfera
    surf(ax,X, Y, Z, C, ...
        'FaceColor', 'interp', ...
        'EdgeColor', 'none', ...
        'FaceLighting', 'gouraud', ...
        'AmbientStrength', 0.5);
    

    % Etichetta (se richiesta)
    if ~isempty(label)
        text(position(1) + radius * 1.2, ...
             position(2) + radius * 1.2, ...
             position(3), ...
             label, ...
             'FontSize', 8, ...
             'FontWeight', 'bold', ...
             'HorizontalAlignment', 'left');
    end
end

%% === Helper ===
function rgb = colorSpecToRGB(c)
    switch c
        case 'r', rgb = [1 0 0];
        case 'g', rgb = [0 1 0];
        case 'b', rgb = [0 0 1];
        case 'k', rgb = [0 0 0];
        case 'm', rgb = [1 0 1];
        case 'c', rgb = [0 1 1];
        case 'y', rgb = [1 1 0];
        otherwise, rgb = [0 0 0];
    end
end
