function [T_final, p_contact] = alignTorusTangentialToPlane(T_init, T_plane, P_local)
    % Author: Antonio FIlianoti

    % Allinea il toroide a essere tangente a un piano inclinato.
    % INPUT:
    %   T_init   : trasformazione iniziale del toroide (4×4)
    %   T_plane  : trasformazione del piano inclinato (4×4)
    %   P_local  : punti omogenei (4×N) del toroide in coordinate locali
    % OUTPUT:
    %   T_final   : trasformazione finale (4×4)
    %   p_contact : punto di contatto 3×1 nel sistema globale

    % Inversa del piano e normale globale
    T_inv_plane  = inv(T_plane);
    plane_normal = T_plane(1:3,3);

    % Operatore di traslazione omogenea
    T_tr = @(v) [eye(3), v; zeros(1,3), 1];

    % Funzione che, per uno spostamento lungo la normale (dz), restituisce
    % la minima altezza (Z) dei punti del toroide nello spazio del piano.
    % Se minimaZ < 0, significa penetrazione.
    function minZ = penetrationDepth(dz)
        T_tmp   = T_init * T_tr(dz * plane_normal);
        P_glob  = T_tmp * P_local;
        P_plane = T_inv_plane * P_glob;
        minZ    = min(P_plane(3,:));
    end

    % Obiettivo: ridurre al minimo la penetrazione (voglio che minZ >= 0)
    costFunc = @(dz) max(0, -penetrationDepth(dz));

    % Vincolo non lineare: almeno un punto sul piano => minZ == 0
    function [c, ceq] = tangencyConstraint(dz)
        c   = [];                 % nessuna disuguaglianza aggiuntiva
        ceq = penetrationDepth(dz); % vogliamo ceq == 0
    end

    % Ottimizzazione con fmincon
    opts = optimoptions('fmincon','Display','none');
    dz0  = 0;  % punto di partenza (nessuna traslazione)
    lb   = -1e3; ub = 1e3;
    dz_opt = fmincon(costFunc, dz0, [],[],[],[], lb, ub, @tangencyConstraint, opts);

    % Costruisco la trasformazione finale
    T_final = T_init * T_tr(dz_opt * plane_normal);

    % Calcolo il punto di contatto (minZ == 0)
    P_glob_final  = T_final * P_local;
    P_plane_final = T_inv_plane * P_glob_final;
    [~, idx] = min(P_plane_final(3,:));
    p_contact = P_glob_final(1:3, idx);
end

