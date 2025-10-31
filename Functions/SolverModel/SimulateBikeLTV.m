function Results = SimulateBikeLTV(Bike, Results)
% =========================================================================
% SimulateBikeLTV - Simula il sistema LTV della motocicletta e produce grafici
%
% Sistema:  ẋ(t) = A(vx(t)) * x(t)
%
% OUTPUT salvati in Results.Sim:
%   .t, .x_deg, .dx_deg, .X_interp, .vx_interp, .x0, .Tmax
%
% Colonne (convenzionali):
% 1 = Spostamento laterale
% 2 = Angolo di yaw
% 3 = Integrale forza laterale anteriore → forza = dx(:,3)
% 4 = Integrale forza laterale posteriore → forza = dx(:,4)
% 5 = Angolo di sterzo
% 6 = Angolo di rollio
% =========================================================================

    %% -------------------- Parametri --------------------
    Tmax     = Bike.Input.Tmax;
    nStates  = size(Results.A, 1);
    n_points = Bike.Input.n_points;

    %% -------------------- Condizioni iniziali --------------------
    x0 = Bike.Input.x0;

    %% -------------------- Legge di velocità --------------------
    vx_fun = @(t) Bike.Input.v_start + ...
                  (Bike.Input.v_finish - Bike.Input.v_start) * t / Tmax;

    %% -------------------- Dinamica --------------------
    dyn = @(t, x) interpA(vx_fun(t), Bike, Results) * x;

    %% -------------------- Integrazione --------------------
    opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
    [tSol, xSol] = ode45(dyn, [0 Tmax], x0, opts);

    %% -------------------- Interpolazione su vx_values --------------------
    vx_interp = linspace(Bike.Input.v_start, Bike.Input.v_finish, n_points);
    t_interp  = (vx_interp - Bike.Input.v_start) / ...
                (Bike.Input.v_finish - Bike.Input.v_start) * Tmax;

    X_interp = zeros(n_points, nStates);
    for i = 1:nStates
        X_interp(:,i) = interp1(tSol, xSol(:,i), t_interp, 'linear', 'extrap');
    end

    %% -------------------- Derivate numeriche --------------------
    t  = tSol(:);
    Nt = length(t);
    dx = zeros(size(xSol));

    if Nt >= 3
        for k = 2:Nt-1
            dx(k,:) = (xSol(k+1,:) - xSol(k-1,:)) / (t(k+1) - t(k-1));
        end
        dx(1,:)    = (xSol(2,:) - xSol(1,:)) / (t(2) - t(1));
        dx(end,:)  = (xSol(end,:) - xSol(end-1,:)) / (t(end) - t(end-1));
    elseif Nt == 2
        dx(:,:) = (xSol(2,:) - xSol(1,:)) / (t(2)-t(1));
    end

    %% -------------------- Conversione in gradi --------------------
    radCols   = intersect([2,5,6], 1:nStates);
    forceCols = intersect([3,4], 1:nStates);

    xSol_deg = xSol;
    dx_deg   = dx;
    if ~isempty(radCols)
        xSol_deg(:,radCols) = rad2deg(xSol(:,radCols));
        dx_deg(:,radCols)   = rad2deg(dx(:,radCols));
    end

    %% -------------------- Salvataggio in Results.Sim --------------------
    Results.Sim.t       = tSol;
    Results.Sim.x_deg   = xSol_deg;
    Results.Sim.dx_deg  = dx_deg;
    Results.Sim.x       = xSol;
    Results.Sim.dx      = dx;
    Results.Sim.X_interp= X_interp;
    Results.Sim.vx_interp = vx_interp;
    Results.Sim.x0      = x0;
    Results.Sim.Tmax    = Tmax;

    %% -------------------- Plot dei risultati --------------------
    plotBikeSimulation(Results.Sim, radCols, forceCols);

    %% =====================================================================
    % Interpolazione matrice A in funzione della velocità
    %% =====================================================================
    function A_out = interpA(vx, Bike, Results)
        A_out = zeros(size(Results.A(:,:,1)));
        for i = 1:size(A_out,1)
            for j = 1:size(A_out,2)
                A_out(i,j) = interp1(Bike.Input.vx_values, ...
                                     squeeze(Results.A(i,j,:)), ...
                                     vx, 'linear', 'extrap');
            end
        end
    end

    %% =====================================================================
    % Funzione di plotting elegante e leggibile
    %% =====================================================================
    function plotBikeSimulation(Sim, radCols, forceCols)
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

        figure('Name','Simulazione LTV Moto', ...
               'Color','w', ...
               'Units','normalized', ...
               'Position',[0.05 0.05 0.9 0.85]);

        tiledlayout(nRows, nCols, 'Padding','compact', 'TileSpacing','compact');

        for i = 1:nPlt
            nexttile;
            hold on;

            % Se è una forza → mostra solo dx (forza)
            if ismember(i, forceCols)
                plot(t, dx(:,i), 'LineWidth', 1.5);
                ylabel(sprintf('%s %s', signalNames{i}, units{i}), 'FontSize', 10);
                xlabel('Tempo [s]');
                title(signalNames{i}, 'FontWeight','normal');
                grid on;
                legend('Forza', 'Location','best'); 
                legend boxoff;
                continue;
            end

            % Altrimenti doppio asse (segnale + derivata)
            yyaxis left
            plot(t, x(:,i), 'LineWidth', 1.5);
            ylabel(sprintf('%s %s', signalNames{i}, units{i}), 'FontSize', 10);

            yyaxis right
            plot(t, dx(:,i), 'LineWidth', 1.3); 
            ylabel('Derivata', 'FontSize', 10);

            xlabel('Tempo [s]');
            title(signalNames{i}, 'FontWeight','normal');
            grid on;
            legend({'Segnale','Derivata'}, 'Location','best'); 
            legend boxoff;
        end

        sgtitle('Evoluzione temporale degli stati - Modello LTV Moto', 'FontWeight','bold');
    end

end
