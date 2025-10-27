function [offset] = calcOffsetFromHT(H, G)
% calcOffsetFromHT Calcola offset del baricentro dall'asse di sterzo
%   H: 4x4 matrice omogenea (rotation | translation)
%   G: 1x3 o 3x1 vettore coordinate baricentro [x y z]
%   useFrameForward (optional): true (default) usa l'asse X del frame H
%                              false usa [1 0 0] globale come "forward"
%
% Output:
%   offset         : distanza scalare (>=0) punto->retta
%   offset_signed  : valore firmato (>0 destra, <0 sinistra) o NaN se non definito
%   P              : punto (1x3) proiezione ortogonale di G sull'asse
%   d              : vettore offset = G - P
%   t              : parametro scalare (coordinata lungo l'asse rispetto all'origine A)

% normalizza input
G = G(:); % colonna 3x1
A = H(1:3,4);         % origine dell'asse (punto A)
R = H(1:3,1:3);       % rotazione (colonne = assi del frame)
u = R(:,3);           % direzione dell'asse: colonna 3 (assunto)
u = u / norm(u);      % vettore unitario direzione asse

% proiezione
w = G - A;
t = dot(w, u);        % coordinata lungo l'asse
P = A + t * u;        % punto più vicino sull'asse
d = G - P;            % vettore offset
offset = norm(d);     % distanza perpendicolare

end