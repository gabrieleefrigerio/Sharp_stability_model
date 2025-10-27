function [Rx, Ry, Rz, T_rot, T_tr] = getOperators()
    % Author: Antonio Filianoti
    % getOperators  Restituisce operatori anonimi per rotazioni e traslazioni
    %
    %   [Rx, Ry, Rz, T_rot, T_tr] = getOperators()
    %
    % OUTPUT:
    %   Rx(theta)  — matrice di rotazione 3×3 attorno all’asse X
    %   Ry(theta)  — matrice di rotazione 3×3 attorno all’asse Y
    %   Rz(theta)  — matrice di rotazione 3×3 attorno all’asse Z
    %   T_rot(R)   — estende una R 3×3 a 4×4 inserendo zeri e 1
    %   T_tr(v)    — matrice 4×4 di traslazione di un vettore v (3×1)

    % Rotazioni elementari in 3×3
    Rx = @(theta) [
        1,          0,           0;
        0, cos(theta), -sin(theta);
        0, sin(theta),  cos(theta)
    ];
    Ry = @(theta) [
         cos(theta), 0, sin(theta);
                  0, 1,          0;
        -sin(theta), 0, cos(theta)
    ];
    Rz = @(theta) [
        cos(theta), -sin(theta), 0;
        sin(theta),  cos(theta), 0;
                 0,           0, 1
    ];

    % Eleviamo a omogenee 4×4
    T_rot = @(R) [R, zeros(3,1); 0 0 0 1];
    T_tr  = @(v) [eye(3), v(:);   0 0 0 1];
end
