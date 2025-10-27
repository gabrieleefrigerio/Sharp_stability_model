function [] = PlotResults(Results, Input)
%--------------------------------------------------------------------------
% PLOTRESULTS - Visualizza i risultati dell’analisi modale e di stabilità.
%
% DESCRIZIONE:
%   La funzione genera tre figure principali:
%       1. Frequenze naturali vs velocità.
%       2. Root Locus dei poli del sistema.
%       3. Parte reale dei poli vs velocità (per analisi di stabilità).
%
%   Inoltre, stampa nel Command Window un riepilogo della stabilità di
%   ciascun modo (stabile/instabile e velocità critica di instabilità).
%
% INPUT:
%   Results [struct] - Struttura contenente i risultati del calcolo:
%       • Results.n_modi              → numero di modi del sistema
%       • Results.D                   → matrice poli (complessi)
%       • Results.frequenze_naturali  → frequenze naturali [Hz]
%
%   Input [struct] - Struttura contenente i parametri di input:
%       • Input.vx_values → vettore delle velocità [m/s]
%
% OUTPUT:
%   Nessuno (solo grafici e messaggi a schermo)
%
%--------------------------------------------------------------------------

    % ---------------- Palette Colori ----------------
    colors = lines(Results.n_modi);     % palette MATLAB predefinita
    poli   = Results.D;                 % poli complessi (matrice n_modi x n_points)

    %% ===============================================================
    %  1) FREQUENZE NATURALI VS VELOCITÀ
    % ================================================================
    figure('Name', 'Frequenze Naturali', 'NumberTitle', 'off');
    hold on; grid on; grid minor;

    for j = 1:Results.n_modi
        plot(Input.vx_values * 3.6, Results.frequenze_naturali(j, :), ...
             'LineWidth', 1.8, 'Color', colors(j, :));
    end

    xlabel('Velocità [km/h]');
    ylabel('Frequenza [Hz]');
    title('Frequenze Naturali vs Velocità');
    legend(arrayfun(@(j) sprintf('Modo %d', j), 1:Results.n_modi, 'UniformOutput', false), ...
           'Location', 'best');

    %% ===============================================================
    %  2) ROOT LOCUS
    % ================================================================
    figure('Name', 'Root Locus', 'NumberTitle', 'off');
    hold on; grid on; grid minor;

    % Traccia i percorsi dei poli nel piano complesso
    for j = 1:Results.n_modi
        h(j) = plot(real(poli(j, :)), imag(poli(j, :)), ...
                    'LineWidth', 1.5, 'Color', colors(j, :));
    end

    % Indica il punto iniziale di ciascun modo con una "x"
    for j = 1:Results.n_modi
        plot(real(poli(j, 1)), imag(poli(j, 1)), ...
             'x', 'MarkerSize', 10, 'LineWidth', 2, 'Color', colors(j, :));
    end

    xlabel('Parte Reale');
    ylabel('Parte Immaginaria');
    title('Root Locus dei Poli');

    % Legenda: modi + simbolo punto iniziale
    legend([h, plot(nan, nan, 'xk', 'MarkerSize', 10, 'LineWidth', 2)], ...
           [arrayfun(@(j) sprintf('Modo %d', j), 1:Results.n_modi, 'UniformOutput', false), ...
            {'Punto iniziale polo'}], ...
           'Location', 'best');

    %% ===============================================================
    %  3) PARTE REALE DEI POLI VS VELOCITÀ
    % ================================================================
    figure('Name', 'Parte Reale dei Poli vs Velocità', 'NumberTitle', 'off');
    hold on; grid on; grid minor;

    for j = 1:Results.n_modi
        plot(Input.vx_values * 3.6, real(poli(j, :)), ...
             'LineWidth', 1.8, 'Color', colors(j, :));
    end

    xlabel('Velocità [km/h]');
    ylabel('Parte Reale (\sigma)');
    title('Parte Reale dei Poli vs Velocità');

    % Linea orizzontale che indica il confine di stabilità
    yline(0, '--k', 'LineWidth', 1.2);

    legend(arrayfun(@(j) sprintf('Modo %d', j), 1:Results.n_modi, 'UniformOutput', false), ...
           'Location', 'best');

    %% ===============================================================
    %  4) ANALISI DI STABILITÀ
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

end
