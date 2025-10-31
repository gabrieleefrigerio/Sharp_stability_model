function [] = PlotResults(Results, Input)
%--------------------------------------------------------------------------
% PLOTRESULTS - Visualizza i risultati dell’analisi modale e della simulazione.
%
% DESCRIZIONE:
%   Questa funzione produce in modo chiaro e leggibile:
%       1) Le frequenze naturali al variare della velocità.
%       2) Il Root Locus dei poli del sistema.
%       3) La parte reale dei poli (analisi di stabilità).
%       4) I risultati della simulazione LTV (se presenti).
%
% INPUT:
%   Results [struct] - Struttura con risultati del calcolo:
%       • Results.n_modi
%       • Results.D (poli complessi)
%       • Results.frequenze_naturali
%       • Results.Sim (facoltativo, dati simulazione temporale)
%
%   Input [struct] - Struttura con i parametri di input:
%       • Input.vx_values → vettore velocità [m/s]
%
%--------------------------------------------------------------------------

    % ---------------- Stile globale dei grafici ----------------
    set(0, 'DefaultFigureWindowStyle', 'normal', ...
       'DefaultFigureColor', 'w', ...
       'DefaultAxesFontSize', 12, ...
       'DefaultLineLineWidth', 1.6, ...
       'DefaultAxesLineWidth', 1.2, ...
       'DefaultAxesBox', 'on', ...
       'DefaultAxesFontName', 'Helvetica', ...
       'DefaultFigurePosition',[50 50 1800 900]);


    % Palette colori base
    colors = lines(Results.n_modi);
    poli   = Results.D;

    %% ===============================================================
    %  1) FREQUENZE NATURALI VS VELOCITÀ
    % ================================================================
    figure('Name', 'Frequenze Naturali', 'NumberTitle', 'off');
    hold on; grid on; grid minor;

    for j = 1:Results.n_modi
        plot(Input.vx_values * 3.6, Results.frequenze_naturali(j, :), ...
             'Color', colors(j, :));
    end

    xlabel('Velocità [km/h]');
    ylabel('Frequenza [Hz]');
    title('Frequenze Naturali vs Velocità', 'FontWeight', 'bold');
    legend(arrayfun(@(j) sprintf('Modo %d', j), 1:Results.n_modi, 'UniformOutput', false), ...
           'Location', 'northwest', 'Box', 'off');
    set(gca, 'FontSize', 11, 'LineWidth', 1);
    hold off;

    %% ===============================================================
    %  2) ROOT LOCUS
    % ================================================================
    figure('Name', 'Root Locus', 'NumberTitle', 'off');
    hold on; grid on; grid minor;

    for j = 1:Results.n_modi
        h(j) = plot(real(poli(j, :)), imag(poli(j, :)), ...
                    'Color', colors(j, :));
    end

    % Segnala il punto iniziale di ciascun modo
    for j = 1:Results.n_modi
        plot(real(poli(j, 1)), imag(poli(j, 1)), ...
             'x', 'MarkerSize', 9, 'LineWidth', 1.5, 'Color', colors(j, :));
    end

    xlabel('Parte Reale (\sigma)');
    ylabel('Parte Immaginaria (\omega)');
    title('Root Locus dei Poli', 'FontWeight', 'bold');
    legend([h, plot(nan, nan, 'xk', 'MarkerSize', 9, 'LineWidth', 1.5)], ...
           [arrayfun(@(j) sprintf('Modo %d', j), 1:Results.n_modi, 'UniformOutput', false), ...
            {'Punto iniziale polo'}], ...
           'Location', 'best', 'Box', 'off');
    hold off;

    %% ===============================================================
    %  3) PARTE REALE DEI POLI VS VELOCITÀ (Analisi Stabilità)
    % ================================================================
    figure('Name', 'Parte Reale dei Poli vs Velocità', 'NumberTitle', 'off');
    hold on; grid on; grid minor;

    for j = 1:Results.n_modi
        plot(Input.vx_values * 3.6, real(poli(j, :)), 'Color', colors(j, :));
    end

    xlabel('Velocità [km/h]');
    ylabel('Parte Reale (\sigma)');
    title('Parte Reale dei Poli vs Velocità', 'FontWeight', 'bold');
    yline(0, '--k', 'LineWidth', 1.2);
    legend(arrayfun(@(j) sprintf('Modo %d', j), 1:Results.n_modi, 'UniformOutput', false), ...
           'Location', 'best', 'Box', 'off');
    hold off;

    %% ===============================================================
    %  4) ANALISI DI STABILITÀ TESTUALE
    % ================================================================
    fprintf('\n--- Analisi stabilità dei modi ---\n');
    for j = 1:Results.n_modi
        parte_reale = real(poli(j, :));
        idx_instabile = find(parte_reale > 0, 1);  % primo punto dove σ > 0

        if isempty(idx_instabile)
            fprintf('Modo %d: sempre STABILE.\n', j);
        elseif idx_instabile == 1
            fprintf('Modo %d: sempre INSTABILE.\n', j);
        else
            fprintf('Modo %d: diventa INSTABILE a vx = %.2f km/h\n', ...
                    j, Input.vx_values(idx_instabile) * 3.6);
        end
    end

    %% ===============================================================
    %  5) RISULTATI DELLA SIMULAZIONE (se presenti)
    % ================================================================
    if isfield(Results, 'Sim') && isstruct(Results.Sim)
        fprintf('\n--- Plot simulazione temporale ---\n');

        % Colonne in cui si trovano forze e angoli
        forceCols = [3, 4];  % Forze anteriori e posteriori
        radCols   = [2, 5, 6]; % Stati angolari in gradi

        % Plot della simulazione con funzione elegante
        plotBikeSimulation(Results.Sim, radCols, forceCols);
    end

end


% =====================================================================
% Funzione di plotting elegante e leggibile (interna a PlotResults)
% =====================================================================
function plotBikeSimulation(Sim, radCols, forceCols)
    % Estrai dati
    t = Sim.t(:);
    x = Sim.x_deg;
    dx = Sim.dx_deg;
    nStates = size(x,2);

    % Etichette e unità
    signalNames = {
        'Spostamento laterale', ...
        'Angolo di yaw', ...
        'Forza laterale anteriore', ...
        'Forza laterale posteriore', ...
        'Angolo di sterzo', ...
        'Angolo di rollio'
        };

    units = {'[m]','[deg]','[N]','[N]','[deg]','[deg]'};

    nPlt = min(nStates, 6);
    nRows = ceil(nPlt/2);
    nCols = 2;

    figure('Name','Simulazione LTV Moto','Color','w','Units','normalized');
    tiledlayout(nRows, nCols, 'Padding','compact', 'TileSpacing','compact');

    for i = 1:nPlt
        nexttile;
        hold on;

        % Se è una forza → mostra solo dx (forza)
        if ismember(i, forceCols)
            plot(t, dx(:,i), 'LineWidth', 1.6);
            ylabel(sprintf('%s %s', signalNames{i}, units{i}), 'FontSize', 10);
            xlabel('Tempo [s]');
            title(signalNames{i}, 'FontWeight','normal');
            grid on; grid minor;
            legend('Forza', 'Location','best'); legend boxoff;
            continue;
        end

        % Altrimenti doppio asse (segnale + derivata)
        yyaxis left
        plot(t, x(:,i), 'LineWidth', 1.6);
        ylabel(sprintf('%s %s', signalNames{i}, units{i}), 'FontSize', 10);

        yyaxis right
        plot(t, dx(:,i), 'LineWidth', 1.2);
        ylabel('Derivata', 'FontSize', 10);

        xlabel('Tempo [s]');
        title(signalNames{i}, 'FontWeight','normal');
        grid on; grid minor;
        legend({'Segnale','Derivata'}, 'Location','best'); legend boxoff;
    end

    sgtitle('Evoluzione temporale degli stati - Modello LTV Moto', 'FontWeight','bold');
end
