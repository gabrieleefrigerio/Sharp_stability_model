function [P1, P2] = chainTangentPoints(C1, r1, C2, r2, side)
% chainTangentPoints - Calcola i punti di tangenza della catena su pignone e corona
%
% Questa funzione determina i punti di contatto della catena su due cerchi
% (ad esempio pignone e corona) giacenti sul piano X-Z, conoscendone i
% centri e i raggi. 
% Permette di distinguere tra tangente superiore e tangente inferiore.
%
% INPUT:
%   C1   - Centro del primo cerchio [x; y; z] (es. pignone)
%   r1   - Raggio del primo cerchio
%   C2   - Centro del secondo cerchio [x; y; z] (es. corona)
%   r2   - Raggio del secondo cerchio
%   side - +1 → tangente superiore
%          -1 → tangente inferiore
%
% OUTPUT:
%   P1 - Punto di tangenza sul primo cerchio (3x1)
%   P2 - Punto di tangenza sul secondo cerchio (3x1)
%
% NOTE:
%   - I calcoli vengono eseguiti nel piano X-Z (coordinate Y costanti).
%   - Utile per tracciare la catena in un modello 3D di trasmissione.

    % === Distanza tra i centri (solo sul piano X-Z) ===
    d = norm(C2([1 3]) - C1([1 3]));  
    
    % Angolo tra i due centri nel piano X-Z
    alpha = atan2(C2(3) - C1(3), C2(1) - C1(1));
    
    % Angolo di correzione dovuto alla differenza di raggi
    beta = acos((r1 - r2) / d);
    
    % Angoli per i punti di tangenza sui due cerchi
    theta1 = alpha + side * beta;  
    theta2 = alpha + side * beta;  
    
    % Punto di tangenza sul cerchio 1 (pignone)
    P1 = [C1(1) + r1*cos(theta1); ...
          C1(2); ...
          C1(3) + r1*sin(theta1)];
      
    % Punto di tangenza sul cerchio 2 (corona)
    P2 = [C2(1) + r2*cos(theta2); ...
          C2(2); ...
          C2(3) + r2*sin(theta2)];
end


