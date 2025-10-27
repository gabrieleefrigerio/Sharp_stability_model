function [V_unique, idx_keep] = removeDuplicateModes(V, tol)
% REMOVEDUPLICATEMODES - Elimina colonne duplicate o coniugate complesse.
%
% INPUT:
%   V   : matrice [n x m] (autovettori complessi)
%   tol : tolleranza per confronto (default = 1e-10)
%
% OUTPUT:
%   V_unique : matrice con colonne uniche
%   idx_keep : indici delle colonne mantenute

if nargin < 2
    tol = 1e-10;
end

n = size(V,2);
keep = true(1, n);

for i = 1:n
    if ~keep(i), continue; end
    vi = V(:,i);

    for j = i+1:n
        vj = V(:,j);

        % Normalizza fase (diretta)
        phase = angle(vi' * vj);
        vj_adj = vj * exp(-1j * phase);

        % Normalizza fase (coniugata)
        phase_conj = angle(vi' * conj(vj));
        vj_conj = conj(vj) * exp(-1j * phase_conj);

        % Se colonne molto simili, elimina la j-esima
        if norm(vi - vj_adj) < tol || norm(vi - vj_conj) < tol
            keep(j) = false;
        end
    end
end

V_unique = V(:, keep);
idx_keep = find(keep);
end
