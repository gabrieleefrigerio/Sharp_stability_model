function p_new = projectToBikePlane(p, T_new)
% projectToBikePlane - Proietta un punto in un nuovo sistema di riferimento
%
% Questa funzione prende un punto espresso nel sistema globale e lo
% riporta (proietta) in coordinate locali rispetto a un frame dato.
%
% INPUT:
%   p     - Punto nello spazio globale (3x1)
%   T_new - Matrice di trasformazione omogenea (4x4) che definisce il nuovo
%           sistema di riferimento (rotazione + traslazione)
%
% OUTPUT:
%   p_new - Coordinate del punto nel sistema locale T_new (3x1)
%
% NOTE:
%   - Utile per riportare tutti i punti della moto in un piano di
%     simmetria traslato.
%   - La funzione calcola l’inversa della trasformazione T_new e
%     la applica al punto.

    % --- Estrai rotazione e traslazione dal frame ---
    R = T_new(1:3,1:3);  % matrice di rotazione
    t = T_new(1:3,4);    % vettore di traslazione

    % --- Calcola l’inversa del frame ---
    Rinv = R.';          % inversa di R (rotazione ortogonale → trasposta)
    tinv = -Rinv * t;    % traslazione inversa
    Tinv = [Rinv tinv;   % matrice omogenea inversa
            0 0 0 1];

    % --- Porta il punto in coordinate omogenee ---
    p_h = [p; 1];

    % --- Applica la trasformazione inversa ---
    p_new_h = Tinv * p_h;

    % --- Estraggo il punto proiettato (solo parte 3D) ---
    p_new = p_new_h(1:3);
end


