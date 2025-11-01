function [Mesh_out] = drawTorus(ax,T, Mesh_in,filled)
    % Author: Antonio Filianoti

    % drawTorus  Genera (e opzionalmente disegna) un toroide 3D
    %
    %   [P, X, Y, Z] = drawTorus(T, R, r, filled, coarse)
    %
    % INPUT:
    %   T       — trasformazione omogenea 4×4 da applicare al toroide
    %   R       — raggio maggiore del toroide
    %   r       — raggio minore (sezione) del toroide
    %   filled  — true = disegna superficie, false = solo genera P
    %   coarse  — true = griglia più grossolana (meno punti)
    %
    % OUTPUT:
    %   P       — punti omogenei 4×N trasformati
    %   X, Y, Z — coordinate 3D parametriche (prima di T)

    % Default argomenti
    if nargin < 4, filled = false;  end

    

    % Applico la trasformazione
    if ~isequal(T, eye(4))
        Mesh_out = T * Mesh_in;
    end

    % Se richiesto, disegno la superficie
    if filled
        % Recupero dimensioni originali
        
        nPoints = size(Mesh_in, 2);
        N = sqrt(nPoints);   % supponendo griglia NxN

        % Ricostruisco coordinate parametriche prima della trasformazione
        X = reshape(Mesh_in(1,:), [N, N]);
        Y = reshape(Mesh_in(2,:), [N, N]);
        Z = reshape(Mesh_in(3,:), [N, N]);
        
        Xw = reshape(Mesh_out(1,:), size(X));
        Yw = reshape(Mesh_out(2,:), size(Y));
        Zw = reshape(Mesh_out(3,:), size(Z));
        surf(ax,Xw, Yw, Zw, ...
             'FaceColor', [0.1 0.1 0.1], ...
             'EdgeColor', 'none', ...
             'FaceAlpha', 0.9);
    end
end
