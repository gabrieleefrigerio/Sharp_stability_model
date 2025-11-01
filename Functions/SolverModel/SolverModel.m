function [Results] = SolverModel(Bike)
% =========================================================================
% SolverModel - Calcola e organizza i risultati modali del modello dinamico
%
% INPUT:
%   Bike -> struttura dati della motocicletta (parametri fisici e cinematici)
%
% OUTPUT:
%   Results -> struttura con:
%       A                        : matrici di stato per ogni condizione
%       V, D                     : autovettori e autovalori non filtrati
%       V_filt, D_filt           : autovettori e autovalori filtrati e normalizzati
%       frequenze_naturali       : vettore delle frequenze proprie (Hz)
%       smorzamenti              : fattori di smorzamento
%       stabilita                : stabilità ("Si"/"No")
% =========================================================================

    % ----------------- Calcolo iniziale per dimensionamento -----------------
    Results = struct();
    Results.A = Sharp_Model(Bike, 1);
    [~, D] = eig(Results.A);
    Results.n_modi = size(D, 1);

    % ----------------- Preallocazioni -----------------
    nPoints = Bike.Input.n_points;
    Results.frequenze_naturali = zeros(Results.n_modi, nPoints);
    Results.smorzamenti        = zeros(Results.n_modi, nPoints);
    Results.stabilita          = strings(Results.n_modi, nPoints);
    Results.A                  = zeros(Results.n_modi, Results.n_modi, nPoints);

    % ----------------- Calcolo matrici di stato -----------------
    for i = 1:nPoints
        Results.A(:,:,i) = Sharp_Model(Bike, i);
    end

    % ----------------- Analisi agli autovalori -----------------
    [Results.V, Results.D] = eigenshuffle(Results.A);

    % ----------------- Calcolo grandezze dinamiche -----------------
    for i = 1:nPoints
        for j = 1:Results.n_modi
            lambda = Results.D(j, i);         % autovalore complesso
            sigma  = real(lambda);             % parte reale (smorzamento)
            omega  = abs(imag(lambda));        % parte immaginaria (pulsazione)
            omega_n = abs(lambda);             % modulo complessivo
            zeta  = (omega_n == 0) * 1 + (omega_n ~= 0) * (-sigma / omega_n);
            freq_n = omega / (2 * pi);         % frequenza naturale [Hz]
    
            Results.frequenze_naturali(j, i) = freq_n;
            Results.smorzamenti(j, i)        = zeta;
            if sigma < 0
                Results.stabilita(j, i) = "Si";
            else
                Results.stabilita(j, i) = "No";
            end
        end
    end

    % ======================================================================
    %  FASE 2: NORMALIZZAZIONE E FILTRAGGIO MODI (eliminazione complessi coniugati)
    % ======================================================================

    [Results.V_filt, Results.D_filt, Results.freq_filt] = filterModesByFrequency(Results.V, Results.D);

    fprintf('✅ Filtraggio completato: modi normalizzati e duplicati rimossi.\n');

    % ======================================================================
    %  (Facoltativo) Conversione cell array -> 3D array se le dimensioni coincidono
    % ======================================================================
    try
        if all(cellfun(@(x) size(x,2), Results.V_filt) == size(Results.V_filt{1},2))
            Results.V_filt = cat(3, Results.V_filt{:});
            Results.D_filt = cat(2, Results.D_filt{:});
        end
    catch
        warning('⚠️ Le dimensioni dei modi filtrati non sono uniformi: mantenuto formato cell array.');
    end

    fprintf('📦 Risultato finale salvato in Results.V_filt e Results.D_filt\n\n');

% function [V_filt, D_filt, freq_filt] = filterModesByFrequency(V, D)
% % =========================================================================
% % filterModesByFrequency - Rimuove modi duplicati in base alla frequenza propria
% %
% % INPUT:
% %   V [nDOF x nModes x nVel]  -> autovettori complessi
% %   D [nModes x nVel]          -> autovalori complessi (lambda)
% %
% % OUTPUT:
% %   V_filt {1 x nVel}  -> autovettori filtrati (una matrice per velocità)
% %   D_filt {1 x nVel}  -> autovalori filtrati coerenti
% %   freq_filt {1 x nVel} -> frequenze filtrate (Hz)
% %
% % CRITERIO:
% %   - Se due modi hanno la stessa frequenza (entro tolleranza 1e-3 Hz),
% %     viene mantenuto solo il primo.
% % =========================================================================
% 
% tol = 1e-3; % tolleranza in Hz per considerare due modi uguali
% [nDOF, nModes, nVel] = size(V);
% 
% V_filt = cell(1, nVel);
% D_filt = cell(1, nVel);
% freq_filt = cell(1, nVel);
% 
% for i = 1:nVel
%     Vi = V(:,:,i);
%     Di = D(:,i);
% 
%     % 1️⃣ Normalizza ogni colonna
%     for j = 1:nModes
%         nrm = norm(Vi(:,j));
%         if nrm > 0
%             Vi(:,j) = Vi(:,j) / nrm;
%         end
%     end
% 
%     % 2️⃣ Calcola frequenze proprie (Hz)
%     freqHz = abs(imag(Di)) / (2*pi);
% 
%     % 3️⃣ Rimuovi modi con frequenze duplicate
%     keep = true(size(freqHz));
%     for j = 2:length(freqHz)
%         if any(abs(freqHz(j) - freqHz(1:j-1)) < tol)
%             keep(j) = false; % è duplicato di un modo precedente
%         end
%     end
% 
%     Vi = Vi(:, keep);
%     Di = Di(keep);
%     freqHz = freqHz(keep);
% 
%     % 4️⃣ Salva risultati filtrati
%     V_filt{i} = Vi;
%     D_filt{i} = Di;
%     freq_filt{i} = freqHz;
% end
% 
% end

function [V_filt, D_filt, freq_filt] = filterModesByFrequency(V, D)
% =========================================================================
% filterModesByFrequency - Seleziona colonne specifiche e normalizza i modi
%
% INPUT:
%   V [nDOF x nModes x nVel]  -> autovettori complessi
%   D [nModes x nVel]         -> autovalori complessi (lambda)
%
% OUTPUT:
%   V_filt {1 x nVel}   -> celle con autovettori filtrati per velocità
%   D_filt {1 x nVel}   -> celle con autovalori filtrati
%   freq_filt {1 x nVel} -> celle con frequenze proprie in Hz
%
% NOTE:
%   - Mantiene solo le colonne [1,6,8,10,12]
%   - Normalizza ogni colonna
%   - Non rimuove duplicati
% =========================================================================

cols_to_keep = [1,6,8,10,12];  % colonne da selezionare
[nDOF, nModes, nVel] = size(V);

V_filt = cell(1, nVel);
D_filt = cell(1, nVel);
freq_filt = cell(1, nVel);

for i = 1:nVel
    Vi = V(:,:,i);
    Di = D(:,i);

    % 🔹 Seleziona solo le colonne desiderate (evita colonne fuori range)
    cols_sel = cols_to_keep(cols_to_keep <= nModes);
    Vi = Vi(:, cols_sel);
    Di = Di(cols_sel);

    % 1️⃣ Normalizza ogni colonna
    for j = 1:size(Vi,2)
        nrm = norm(Vi(:,j));
        if nrm > 0
            Vi(:,j) = Vi(:,j) / nrm;
        end
    end

    % 2️⃣ Calcola frequenze proprie (Hz)
    freqHz = abs(imag(Di)) / (2*pi);

    % 3️⃣ Rimozione frequenze duplicate (commentata)
    tol = 1e-3;
    keep = true(size(freqHz));
    for j = 2:length(freqHz)
        if any(abs(freqHz(j) - freqHz(1:j-1)) < tol)
            keep(j) = false;
        end
    end
    Vi = Vi(:, keep);
    Di = Di(keep);
    freqHz = freqHz(keep);

    % 4️⃣ Salva risultati filtrati nelle celle
    V_filt{i} = Vi;
    D_filt{i} = Di(:);
    freq_filt{i} = freqHz(:);
end



end






end
