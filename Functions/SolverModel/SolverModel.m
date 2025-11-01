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
    Results.Speed = linspace(Bike.Input.v_start, Bike.Input.v_finish, Bike.Input.n_points)*3.6;
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
    %  FASE 3: CREAZIONE DELLA SOTTOSTRUTTURA Results.Modes
    % ======================================================================
    
    % Definizione nomi dei modi e colonne corrispondenti in V_filt
    modeNames = {'Wobble', 'Weave', 'RearWobble', 'Capsize'};
    cols       = [1, 2, 3, 4];  % colonne corrispondenti in V_filt
    
    nVel = length(Results.V_filt);
    nDOF = size(Results.V_filt{1},1);
    
    % Inizializzazione struttura
    Results.Modes = struct();
    
    for k = 1:length(modeNames)
        modeName = modeNames{k};
        
        % Preallocazione
        freqVec = NaN(1, nVel);
        modeMat = NaN(nDOF, nVel);
        
        for i = 1:nVel
            Vi = Results.V_filt{i};
            freq_i = Results.freq_filt{i};
            
            % Controllo che la colonna esista
            if cols(k) <= size(Vi,2)
                modeMat(:,i) = Vi(:, cols(k));
                freqVec(i)  = freq_i(cols(k));
            end
        end
        
        % Assegna alla sottostruttura
        Results.Modes.(modeName).mode = modeMat;
        Results.Modes.(modeName).freq = freqVec;
    end
    
    fprintf('📦 Sottostruttura Results.Modes generata con Wobble, Weave, RearWobble e Capsize.\n');
    
    %% ===================== TRACKING FREQUENZE MODALI =====================
    % Descrizione:
    % Aggiorna le frequenze proprie e i modi modali passo-passo verificando:
    % 1) Protezione tratto avanti per frequenze coincidenti
    % 2) Scambio modalità solo se una frequenza è più vicina al suo valore precedente
    dx_prot = 5;       % tratto protetto in unità della velocità (es. Km/h)
tol_coinc = 1;     % tolleranza per considerare due frequenze coincidenti [Hz]

% Richiamo la funzione
Results = TrackModalFrequenciesWithTolerance(Results, dx_prot, tol_coinc);

    
    dx_kmh     = 5;       % tratto protetto [km/h]
    tol_close  = 1;     % tolleranza frequenze coincidenti [Hz]

    Speed      = Results.Speed;
    modeNames  = fieldnames(Results.Modes);
    nModes     = length(modeNames);
    nVel       = length(Speed);

    % Conversione dx_kmh in punti indice
    dx_idx = find(Speed >= Speed(1) + dx_kmh,1) - 1;
    if isempty(dx_idx)
        dx_idx = 1;
    end

    % Sostituisci NaN con 0
    for k = 1:nModes
        Results.Modes.(modeNames{k}).freq(isnan(Results.Modes.(modeNames{k}).freq)) = 0;
    end

    % Loop sulla velocità a partire dal secondo punto
    for iVel = 2:nVel
        prevFreqs = zeros(1,nModes);
        currFreqs = zeros(1,nModes);
        for k = 1:nModes
            prevFreqs(k) = Results.Modes.(modeNames{k}).freq(iVel-1);
            currFreqs(k) = Results.Modes.(modeNames{k}).freq(iVel);
        end

        tempFreq = currFreqs;
        tempMode = zeros(size(Results.Modes.(modeNames{1}).mode,1), nModes);

        for k = 1:nModes
            % ================= Protezione tratto con frequenze coincidenti =================
            xRange = iVel:min(iVel+dx_idx,nVel);
            freqSegment = zeros(length(xRange), nModes);
            for m = 1:nModes
                freqSegment(:,m) = Results.Modes.(modeNames{m}).freq(xRange);
            end
            coincidente = false;
            for m1 = 1:nModes-1
                for m2 = m1+1:nModes
                    if any(abs(freqSegment(:,m1)-freqSegment(:,m2)) < tol_close)
                        coincidente = true;
                        break
                    end
                end
                if coincidente, break; end
            end
            if coincidente
                tempFreq(k) = currFreqs(k);
                tempMode(:,k) = Results.Modes.(modeNames{k}).mode(:,iVel);
                continue
            end

            % ============== Trova frequenza più vicina al precedente ==============
            diffPrev = abs(currFreqs - prevFreqs(k));
            [~, sel] = min(diffPrev);

            % Se la frequenza più vicina è proprio quella corrente → ok
            if sel == k
                tempFreq(k) = currFreqs(k);
                tempMode(:,k) = Results.Modes.(modeNames{k}).mode(:,iVel);
            else
                % Altrimenti scambia con quella più vicina
                tempFreq(k) = currFreqs(sel);
                tempMode(:,k) = Results.Modes.(modeNames{sel}).mode(:,iVel);
                % Segnalo che questa frequenza è stata presa dall’altro modo
                currFreqs(sel) = currFreqs(k);
            end
        end

        % Aggiorna i valori nel passo corrente
        for k = 1:nModes
            Results.Modes.(modeNames{k}).freq(iVel) = tempFreq(k);
            Results.Modes.(modeNames{k}).mode(:,iVel)  = tempMode(:,k);
        end
    end



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
        freq_filt{i} = flip(freqHz(:));
    end

    end




end
