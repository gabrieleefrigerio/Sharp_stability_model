function [Results] = SolverModel(Bike)
% ===============================================================
% SolverModel - Calcolo dei modi propri (autovalori/autovettori)
%                per il modello dinamico della bicicletta.
%
% INPUT:
%   Bike : struttura con i parametri del modello, incl.:
%          ├── Input.n_points → numero di punti di calcolo (velocità ecc.)
%
% OUTPUT:
%   Results : struttura con i risultati modali:
%          ├── A(:,:,i)              → matrice di stato al punto i
%          ├── D_cell{i}             → autovalori complessi
%          ├── V_cell{i}             → autovettori complessi
%          ├── frequenze_naturali{i} → frequenze [Hz]
%          ├── smorzamenti{i}        → fattori di smorzamento [-]
%          ├── stabilita{i}          → "Si"/"No" (stabile/instabile)
%
% NOTE:
%   - Gli autovettori vengono filtrati per rimuovere modi doppi
%     (colonne identiche o coniugate complesse).
%   - Tutte le frequenze sono espresse in [Hz].
%
% Author : Antonio Filianoti
% Date   : 2025-10-27
% ===============================================================

%% === 1️⃣ Inizializzazione ===
Results = struct();

% Primo calcolo (solo per determinare dimensioni)
A0 = Sharp_Model(Bike, 1);
[~, D0] = eig(A0);
n_modes = size(D0, 1);
n_points = Bike.Input.n_points;

% Preallocazione matrici di stato
Results.A = zeros(n_modes, n_modes, n_points);

% Celle per risultati (numero di modi può variare dopo filtraggio)
Results.V_cell = cell(1, n_points);
Results.D_cell = cell(1, n_points);
Results.frequenze_naturali = cell(1, n_points);
Results.smorzamenti = cell(1, n_points);
Results.stabilita = cell(1, n_points);

%% === 2️⃣ Calcolo matrici di stato per ciascun punto ===
for i = 1:n_points
    Results.A(:,:,i) = Sharp_Model(Bike, i);
end

%% === 3️⃣ Calcolo autovalori/autovettori e filtraggio modi ===
tol = 1e-8;  % tolleranza numerica per confronto autovettori

for i = 1:n_points
    % Calcolo autovalori e autovettori
    [V_i, D_i] = eig(Results.A(:,:,i));

    % Converto autovalori in vettore colonna
    D_vec = diag(D_i);

    % --- Rimuovo autovettori doppi (uguali o coniugati) ---
    [V_unique, idx_unique] = removeDuplicateModes(V_i, tol);
    D_unique = D_vec(idx_unique);

    % Salvo risultati filtrati
    Results.V_cell{i} = V_unique;
    Results.D_cell{i} = D_unique;

    % === 4️⃣ Calcolo grandezze modali ===
    nModes_i = numel(D_unique);
    freq_i = zeros(nModes_i, 1);   % [Hz]
    zeta_i = zeros(nModes_i, 1);   % [-]
    stab_i = strings(nModes_i, 1); % ["Si"/"No"]

    for j = 1:nModes_i
        lambda = D_unique(j);
        sigma = real(lambda);        % parte reale → stabilità
        omega = abs(imag(lambda));   % parte immaginaria → pulsazione
        omega_n = abs(lambda);       % modulo complesso

        if omega_n ~= 0
            zeta = -sigma / omega_n; % smorzamento adimensionale [-]
        else
            zeta = 1;
        end

        freq = omega / (2*pi);       % frequenza naturale [Hz]

        freq_i(j) = freq;
        zeta_i(j) = zeta;

        % stabilità: negativa = stabile
        if sigma < 0
            stab_i(j) = "Si";
        else
            stab_i(j) = "No";
        end
    end

    Results.frequenze_naturali{i} = freq_i;
    Results.smorzamenti{i} = zeta_i;
    Results.stabilita{i} = stab_i;
end

end
