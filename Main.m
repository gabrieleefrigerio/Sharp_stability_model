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
Results = SimulateBikeLTV(Bike, Results);

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
PlotBikeMode(Bike, P, Results.V(:,1,50));

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

function PlotBikeMode(Bike, P, ModeShape)
% =========================================================================
% PlotBikeMode - Animazione statica della moto deformata da un modo modale
%
% INPUT:
%   Bike       -> struttura dati della moto
%   P          -> struttura punti cinematici (da getBike)
%   ModeShape  -> vettore del modo da plottare
%
% NOTE:
%   Tutti gli angoli devono essere in radianti, spostamenti in metri.
% =========================================================================

    % Normalizzo l'autovettore
    ModeShape = abs(ModeShape)/norm(ModeShape);

    % Stato iniziale base [y_dis; yaw; roll; steer;  in m]
    State0 = [0.2; deg2rad(30); deg2rad(60); deg2rad(3)];

    % Componenti rilevanti del modo
    Ampl_factor = [ModeShape(1); ModeShape(2); ModeShape(5); ModeShape(6)];

    % Stato massimo da plottare = base .* fattore * scala
    Scale = 10000;
    StateMax = State0 .* Ampl_factor * Scale;

    % Definisci i 8 frame dell'animazione tra StateMax e -StateMax
    nFrames = 8;
    factorVec = linspace(1, -1, nFrames);

    % Mantieni la stessa figura
    f = figure('Position', [100, 100, 1600, 900]);
    ax = axes('Parent', f);
    view(45,30); axis equal; grid on; camlight; hold on;

    for i = 1:nFrames
        % Stato attuale per questo frame
        State = StateMax * factorVec(i);

        % Calcola matrici e punti aggiornati
        [T, P_upd, Bike_upd] = MultibodyMatrices(Bike, P, State);

        % Pulizia e plot
        cla(ax); camlight;  % cancella solo i contenuti dell'asse, mantiene figure e assi
        PlotMultibody(T, P_upd, Bike_upd, ax);
        title(sprintf('Moto deformata dal modo modale - Frame %d/%d', i, nFrames), ...
            'FontWeight','bold', 'FontSize', 14);

        % disegno ora la figura
        drawnow;
    end

end


%%

M =  Results.V(:,:,50);

% Trova colonne da eliminare
colsToRemove = false(1, size(M,2)); % inizializza vettore booleano

for j = 1:size(M,2)
    col = M(:,j);
    if nnz(col) == 1 && any(col == 1)
        colsToRemove(j) = true;
    end
end

% Elimina le colonne individuate
M(:, colsToRemove) = [];