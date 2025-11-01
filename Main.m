% ----------------- Impostazioni iniziali -----------------
clear; clc; close all;

% ----------------- Import data -----------------
Bike = load_PMF_V2();
[Bike, Output, P, T] = getBike(Bike);
Bike.data = computeData(P,Bike, Output);

%%

% ----------------- Model Inputs -----------------
Bike.Input.v_start = 1; % start speed in km/h
Bike.Input.v_finish = 100; % finish speed in km/h
Bike.Input.n_points = 50; % speed discretization points

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


%%
% Esempio d’uso:
ModeShapes = Results.V_filt{25};
freq_filt = flip(Results.freq_filt{25});
speed_kmh = 50;

% 1️⃣ Per esportare GIF animate
 ExportBikeModeGIF(Bike, P, ModeShapes, freq_filt, speed_kmh, 'Modo');

% 2️⃣ Per plottare i modi statici (tutti insieme)
PlotBikeModeStatic(Bike, P, ModeShapes, freq_filt, speed_kmh);



function PlotBikeModeStatic(Bike, P, ModeShapes, freqsHz, speed_kmh)
% =========================================================================
% PlotBikeModeStatic - Mostra il modello della moto deformato da ciascun
%                      modo modale, in configurazione statica (no animazione)
%
% INPUT:
%   Bike        -> struttura dati della moto
%   P           -> struttura punti cinematici (da getBike)
%   ModeShapes  -> matrice dei modi [nDOF x nModes]
%   freqsHz     -> vettore frequenze proprie [nModes x 1] in Hz
%   speed_kmh   -> velocità della moto in km/h (scalare)
% =========================================================================

    nModes = size(ModeShapes, 2);
    Scale  = 2500;

    % Stato base [y_dis; yaw; roll; steer]
    State0 = [10; deg2rad(20); deg2rad(90); deg2rad(25)];

    % Crea una sola figura visibile
    f = figure('Visible','on','Position',[100,100,1600,900]);
    t = tiledlayout(f, 2, 2, 'TileSpacing','compact','Padding','compact');

    for imode = 1:nModes
        ModeShape = ModeShapes(:, imode);
        ModeShape = abs(ModeShape)/norm(ModeShape);

        Ampl_factor = [ModeShape(1); ModeShape(2); ModeShape(5); ModeShape(6)];
        StateMax = State0 .* Ampl_factor * Scale;

        [T, P_upd, Bike_upd] = MultibodyMatrices(Bike, P, StateMax);

        % Crea asse per questo modo
        nexttile(t);
        hold on;
        ax = gca;

        % ✅ Assicura che PlotMultibody disegni qui
        axes(ax);
        PlotMultibody(T, P_upd, Bike_upd, ax);

        axis equal; grid on; view(45,30);
        camlight;

        % Titolo informativo
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

        title(sprintf('Modo %d — f = %s Hz — V = %s km/h', ...
            imode, freqStr, velStr), ...
            'FontWeight','bold','FontSize',12);
    end

    sgtitle(f, 'Moto deformata dai modi modali (configurazione statica)', ...
        'FontSize',14,'FontWeight','bold');

end


function ExportBikeModeGIF(Bike, P, ModeShapes, freqsHz, speed_kmh, baseName)
% =========================================================================
% ExportBikeModeGIF - Esporta una GIF per ciascun modo modale
%                     e le salva nella cartella "ResultsGIF"
%
% INPUT:
%   Bike        -> struttura dati della moto
%   P           -> struttura punti cinematici (da getBike)
%   ModeShapes  -> matrice dei modi [nDOF x nModes]
%   freqsHz     -> vettore frequenze proprie [nModes x 1] in Hz
%   speed_kmh   -> velocità della moto in km/h (scalare)
%   baseName    -> prefisso per i file GIF (es. 'Modo')
%
% NOTE:
%   L'animazione normale: 0 → +State0 → -State0 → 0
%   Per modi con freq < 0.5 Hz: 0 → +State0
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
    Scale = 1000;       % amplificazione visuale
    delay = 0.1;        % tempo tra frame (s)
    State0 = [10; deg2rad(20); deg2rad(90); deg2rad(25)];

    % Figura invisibile
    f = figure('Visible','off','Position',[100,100,1600,900]);
    ax = axes('Parent', f);
    view(45,30); axis equal; grid on; camlight; hold on;

    for imode = 1:nModes
        ModeShape = ModeShapes(:, imode);
        ModeShape = abs(ModeShape)/norm(ModeShape);
        Ampl_factor = [ModeShape(1); ModeShape(2); ModeShape(5); ModeShape(6)];
        StateMax = State0 .* Ampl_factor * Scale;

        % 🔹 Percorso GIF
        gifName = sprintf('%s_%02d.gif', baseName, imode);
        gifPath = fullfile(outputFolder, gifName);

        % 🔹 Titolo informativo
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

        % 🔸 Determina il vettore dei frame
        if freqsHz(imode) < 0.5
            % Modi molto lenti: solo 0 → +State0
            N = 10;
            factorVec = linspace(0, 1, N)*10;
        else
            % Animazione normale: 0 → +State0 → -State0 → 0
            N = 2;
            factorVec = [linspace(0, 1, N), linspace(1, -1, 2*N), linspace(-1, 0, N)];
        end

        % 🔸 Ciclo animazione
        for i = 1:length(factorVec)
            State = StateMax * factorVec(i);
            [T, P_upd, Bike_upd] = MultibodyMatrices(Bike, P, State);

            cla(ax); camlight;
            PlotMultibody(T, P_upd, Bike_upd, ax);
            title(sprintf('Modo %d — f = %s Hz — V = %s km/h', ...
                imode, freqStr, velStr), 'FontWeight','bold','FontSize',14);

            % Frame → GIF
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
    disp('🎬 Tutte le GIF generate con successo!');
end


