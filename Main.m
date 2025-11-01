% ----------------- Impostazioni iniziali -----------------
clear; clc; close all;

% ----------------- Import data -----------------
Bike = load_PMF_V2();
[Bike, Output, P, T] = getBike(Bike);
Bike.data = computeData(P,Bike, Output);

%%

% ----------------- Model Inputs -----------------
Bike.Input.v_start = 1; % start speed in km/h
Bike.Input.v_finish = 200; % finish speed in km/h
Bike.Input.n_points = 100; % speed discretization points

% ----------------- Parametri di simulazione LTV -----------------
Bike.Input.Tmax = 10;                 % tempo totale simulazione [s]
Bike.Input.x0 = 1e-3 * ones(12,1);    % condizione iniziale
%Bike.Input.x0(1) = 1e-3;             % piccola perturbazione iniziale

% function to find model input vectors
Bike = ModelInput(Bike);

% Solver Model
Results = SolverModel(Bike);

% ----------------- Simulazione sistema LTV -----------------
% Results = SimulateBikeLTV(Bike, Results);

% Plot Results
PlotResults(Results, Bike.Input);

% animazione
%AnimateBike(Bike, Results, P, 10);

%plot mode animation
PlotBikeMode(Bike, P, Results.V(:,1,50));

%%

% animazione
AnimateBike(Bike, Results, P, 10);

function AnimateBike(Bike, Results, P, nFrames)
% =========================================================================
% AnimateBike - Crea un'animazione 3D della moto lungo la simulazione
%
% INPUT:
%   Bike     -> struttura dati della moto
%   Results  -> struttura con Results.Sim.x (matrice stati) e Results.Sim.t
%   P        -> struttura punti cinematici (da getBike)
%   nFrames  -> numero di frame desiderati nell'animazione
%
% FUNZIONA COSÌ:
%   - Interpola la soluzione temporale su nFrames punti
%   - Per ogni istante ricostruisce le trasformazioni con MultibodyMatrices
%   - Mostra la moto con PlotMultibody
% =========================================================================

    fprintf('🎬 Avvio animazione con %d frame...\n', nFrames);

    % Estrazione tempo e stati
    t_all = Results.Sim.t;
    x_all = Results.Sim.x;

    % Interpolazione in nFrames punti equispaziati
    t_interp = linspace(min(t_all), max(t_all), nFrames);
    x_interp = interp1(t_all, x_all, t_interp);

    % Figura principale
    f = figure('Position', [200, 100, 1200, 800]);

    for i = 1:nFrames
        % Stato attuale (uno per frame)
        State = x_interp(i, :);

        % Aggiorna trasformazioni
        [T, P_upd, Bike_upd] = MultibodyMatrices(Bike, P, State);

        % Pulizia figura
        clf(f);
        hold on;

        % Plot multibody
        PlotMultibody(T, P_upd, Bike_upd, []);

        % Aggiornamento titolo
        title(sprintf('Motorcycle Animation - Frame %d/%d (t = %.2f s)', i, nFrames, t_interp(i)));

        drawnow;
    end

    fprintf('✅ Animazione completata!\n');
end

%%


% Estrai il modo modale e gli autovalori
ModeShapes = Results.V(:,:,50);
Dvec = Results.frequenze_naturali;   % autovalori associati

% Trova colonne da eliminare
colsToRemove = false(1, size(ModeShapes,2)); % inizializza vettore booleano

for j = 1:size(ModeShapes,2)
    col = ModeShapes(:,j);
    % condizione: solo un elemento non nullo e pari a 1
    if nnz(col) == 1 && any(col == 1)
        colsToRemove(j) = true;
    end
end

% Elimina le colonne individuate dai modi
ModeShapes(:, colsToRemove) = [];

% 🔹 Rimuovi anche gli stessi elementi dal vettore degli autovalori
Dvec(colsToRemove) = [];

% 🔹 Calcola le frequenze proprie (in Hz)
% (assumendo che D contenga autovalori = ω^2, con ω in rad/s)
freqsHz = sqrt(Dvec);

% ✅ Ora hai:
%   ModeShapes = modi filtrati
%   freqsHz    = frequenze proprie corrispondenti (in Hz)





PlotBikeMode(Bike, P, ModeShapes, freqsHz, 100 );

% function PlotBikeMode(Bike, P, ModeShape)
% % =========================================================================
% % PlotBikeMode - Plotta la moto deformata da un modo modale (statica)
% %
% % INPUT:
% %   Bike       -> struttura dati della moto
% %   P          -> struttura punti cinematici (da getBike)
% %   ModeShape  -> vettore del modo da plottare
% %
% % NOTE:
% %   Tutti gli angoli devono essere in radianti, spostamenti in metri.
% % =========================================================================
% 
% 
%     % normalizzo l'autovettore
%     ModeShape = abs(ModeShape)/norm(ModeShape);
% 
%     % Stato iniziale base
%     State0 = [ 0.2; deg2rad(20); deg2rad(60); deg2rad(30)];  % [yaw; roll; steer; y_dis in m]
% 
%     Ampl_factor = [ModeShape(1); ModeShape(2); ModeShape(5); ModeShape(6)]; % prendo solo le componenti che mi interessano
% 
%     % Stato finale da plottare = base + modo * fattore
%     State = State0  .* Ampl_factor * 10000;
% 
%     % Calcola matrici e punti aggiornati
%     [T, P_upd, Bike_upd] = MultibodyMatrices(Bike, P, State);
% 
%     % Plot della moto
%     PlotMultibody(T, P_upd, Bike_upd, []);
%     title('Moto deformata dal modo modale', 'FontWeight','bold', 'FontSize', 14);
% 
% end

function PlotBikeMode(Bike, P, ModeShapes, freqsHz, speed_kmh, baseName)
% =========================================================================
% PlotBikeMode - Esporta GIF per ogni modo modale in background
%                e le salva nella cartella "ResultsGIF"
%
% INPUT:
%   Bike        -> struttura dati della moto
%   P           -> struttura punti cinematici (da getBike)
%   ModeShapes  -> matrice dei modi [nDOF x nModes] (ogni colonna = un modo)
%   freqsHz     -> vettore frequenze proprie [nModes x 1] in Hz
%   speed_kmh   -> velocità della moto in km/h (scalare)
%   baseName    -> (opzionale) prefisso per i file GIF (es. 'Modo')
%
% NOTE:
%   L'animazione va da [0 0 0 0] → +State0 → -State0 → [0 0 0 0]
% =========================================================================

    if nargin < 6
        baseName = 'Mode';
    end

    % 🔹 Crea la cartella ResultsGIF se non esiste
    outputFolder = fullfile(pwd, 'ResultsGIF');
    if ~exist(outputFolder, 'dir')
        mkdir(outputFolder);
        fprintf('📁 Creata cartella: %s\n', outputFolder);
    end

    nModes = size(ModeShapes, 2);

    % Parametri animazione
    N = 2; % passi per tratto
    factorVec = [linspace(0, 1, N), linspace(1, -1, 2*N), linspace(-1, 0, N)];
    Scale = 10000;       % amplificazione visuale
    delay = 0.1;         % tempo tra frame (s)

    % Stato base [y_dis; yaw; roll; steer]
    State0 = [0.7; deg2rad(20); deg2rad(60); deg2rad(15)];

    % Crea figura invisibile per velocità
    f = figure('Visible','off','Position',[100,100,1600,900]);
    ax = axes('Parent', f);
    view(45,30); axis equal; grid on; camlight; hold on;

    % Loop sui modi
    for imode = 1:nModes
        ModeShape = ModeShapes(:, imode);
        ModeShape = abs(ModeShape)/norm(ModeShape);

        Ampl_factor = [ModeShape(1); ModeShape(2); ModeShape(5); ModeShape(6)];
        StateMax = State0 .* Ampl_factor * Scale;

        % 🔹 Percorso completo della GIF
        gifName = sprintf('%s_%02d.gif', baseName, imode);
        gifPath = fullfile(outputFolder, gifName);

        % 🔹 Frequenza e velocità per il titolo
        if nargin >= 4 && length(freqsHz) >= imode
            freqStr = sprintf('%.2f', freqsHz(imode));
        else
            freqStr = 'N/A';
        end
        if nargin >= 5 && ~isempty(speed_kmh)
            velStr = sprintf('%.1f', speed_kmh);
        else
            velStr = 'N/A';
        end

        fprintf('▶ Genero %s ... ', gifPath);

        for i = 1:length(factorVec)
            State = StateMax * factorVec(i);

            [T, P_upd, Bike_upd] = MultibodyMatrices(Bike, P, State);

            cla(ax); camlight;
            PlotMultibody(T, P_upd, Bike_upd, ax);

            % 🔹 Titolo informativo
            title(sprintf('Modo %d — f = %s Hz — V = %s km/h', ...
                imode, freqStr, velStr), ...
                'FontWeight','bold','FontSize',14);

            % Cattura frame
            frame = getframe(f);
            im = frame2im(frame);
            [A, map] = rgb2ind(im, 256);

            if i == 1
                imwrite(A, map, gifPath, 'gif', 'LoopCount', Inf, 'DelayTime', delay);
            else
                imwrite(A, map, gifPath, 'gif', 'WriteMode', 'append', 'DelayTime', delay);
            end
        end

        fprintf('✅ Salvata in %s\n', gifPath);
    end

    close(f);
    disp('🎬 Tutte le GIF generate in background con successo!');
end





