function PlotMultibody(T, P, data, ax)
% PlotMultibody - Visualizza la geometria del sistema multibody della moto.
%
% Author: Antonio Filianoti
%
% INPUT:
%   T    - Struttura contenente le matrici di trasformazione omogenee (4x4) di ciascun frame
%   P    - Struttura contenente le posizioni di punti rilevanti nel sistema
%   data - Struttura contenente parametri geometrici della moto (ruote, torus, ecc.)
%
% EFFETTO:
%   Crea una visualizzazione 3D della moto, includendo telaio, ruote, forcelle, baricentri e assi.

    %% === Impostazioni iniziali della figura ===
    if nargin < 4
        f = figure('Position', [100, 100, 1600, 900]);
        ax = axes('Parent', f);
        view(45,30); axis equal; grid on; camlight; hold on;
    end


    %% === Parametri grafici ===
    giallo_ohlins   = [1.0, 0.72, 0.0];      % Colore foderi forcella (tipo Öhlins)
    grigio_cromato  = [0.72, 0.72, 0.72];    % Colore steli forcella
    diameter_fodero = 40;                   % Diametro parte superiore forcella
    diameter_stelo  = 30;                   % Diametro stelo forcella (parte mobile)
    L_axis          = 30;                   % Lunghezza assi frame secondari
    L_axis_main     = 150;                  % Lunghezza assi frame principali

    %% === Disegno del piano strada ===
    drawRoad(ax, T.T_SDR_abs, 1000);  % Piano strada con estensione 1000 mm

    %% === Disegno dei frame ===
    % Frame principali
    % drawFrame(T.T_SDR_abs,         'SDR Abs', L_axis_main);
    % drawFrame(T.T_contatto_post,   'Rear Wheel Contact', L_axis_main);
    % drawFrame(T.T_centro_post,     'Rear Wheel Center', L_axis_main);
    % drawFrame(T.T_swing_pivot,     'Swingarm Pivot', L_axis_main);
    % drawFrame(T.T_steering_axis,   'Steering Axis', L_axis_main);
    % drawFrame(T.T_contatto_ant,    'Front Wheel Contact', L_axis_main);
    % drawFrame(T.T_frame_pivot,     'Frame Pivot', L_axis_main);
    % drawFrame(T.T_pinion,     'Pinion', L_axis_main);
    % drawFrame(T.T_sprocket,     'Sprocket', L_axis_main);

    % Frame secondari (senza etichetta)
    % drawFrame(T.T_sup_forc_dx,     '', L_axis);
    % drawFrame(T.T_sup_forc_sx,     '', L_axis);
    % drawFrame(T.T_mid_forc_dx,     '', L_axis);
    % drawFrame(T.T_mid_forc_sx,     '', L_axis);
    % drawFrame(T.T_end_forc_dx,     '', L_axis);
    % drawFrame(T.T_end_forc_sx,     '', L_axis);
    % drawFrame(T.T_foot_dx,         '', L_axis);
    % drawFrame(T.T_foot_sx,         '', L_axis);
    % drawFrame(T.T_centro_ant,      'Front Wheel Center', L_axis);

    %% === Disegno delle ruote ===
    drawTorus(ax, T.T_centro_post, data.Wheels.CoarseRearMesh, true);   % Ruota posteriore
    drawTorus(ax, T.T_centro_ant,  data.Wheels.CoarseFrontMesh, true);  % Ruota anteriore

    %% === Disegno delle forcelle ===
    % Parte superiore: foderi (gialli)
    drawCylinder(ax, T.T_sup_forc_sx, T.T_mid_forc_sx, diameter_fodero, giallo_ohlins);
    drawCylinder(ax, T.T_sup_forc_dx, T.T_mid_forc_dx, diameter_fodero, giallo_ohlins);

    % Parte inferiore: steli (grigi)
    drawCylinder(ax, T.T_mid_forc_sx, T.T_end_forc_sx, diameter_stelo, grigio_cromato);
    drawCylinder(ax, T.T_mid_forc_dx, T.T_end_forc_dx, diameter_stelo, grigio_cromato);

    %% === Connessioni rigide (linee nere) ===
    % Forcellone (swingarm): dal centro ruota posteriore al pivot
    plot3(ax, [P.p_centro_post(1), P.p_pivot(1)], ...
          [P.p_centro_post(2), P.p_pivot(2)], ...
          [P.p_centro_post(3), P.p_pivot(3)], ...
          '-k', 'LineWidth', 3);

    % Telaio: dal pivot al punto sull’asse sterzo
    plot3(ax, [P.p_pivot(1), P.p_steering_axis(1)], ...
          [P.p_pivot(2), P.p_steering_axis(2)], ...
          [P.p_pivot(3), P.p_steering_axis(3)], ...
          '-k', 'LineWidth', 3);

    %% === Piastra di sterzo superiore ===
    V = [P.p_piastra_sup_dx'; ...
         P.p_piastra_sup_sx'; ...
         P.p_steering_axis'];
    patch(ax, 'Vertices', V, 'Faces', 1:3, ...
          'FaceColor', [0.9, 0.9, 0.9], 'EdgeColor', 'k', 'LineWidth', 1.5);

    

    %% === VISUALIZZAZIONE AVANCORSA (TRAIL) ===
    % % Calcola l’avancorsa usando la funzione dedicata
    % [trail, NormalTrail, p_intersect, Trail_vec] = trailComputation(T, P, Output);
    % 
    % % Disegna l’asse di sterzo esteso fino al piano strada
    % v_axis = T.T_steering_axis(1:3,3);
    % p_axis = P.p_steering_axis;
    % p_axis_far = p_axis + v_axis * (-1000);   % prolunga di 2 metri
    % plot3(ax, [p_axis(1), p_axis_far(1)], ...
    %           [p_axis(2), p_axis_far(2)], ...
    %           [p_axis(3), p_axis_far(3)], ...
    %           '--r', 'LineWidth', 1.5, 'DisplayName','Asse sterzo');
    % 
    % % Punto di intersezione asse sterzo - piano strada
    % plot3(ax, p_intersect(1), p_intersect(2), p_intersect(3), ...
    %           'ro', 'MarkerSize', 8, 'MarkerFaceColor','r', ...
    %           'DisplayName','Intersezione asse sterzo');
    % 
    % % Trail = dal contatto anteriore all’intersezione
    % plot3(ax, [P.p_contatto_ant(1), p_intersect(1)], ...
    %           [P.p_contatto_ant(2), p_intersect(2)], ...
    %           [P.p_contatto_ant(3), p_intersect(3)], ...
    %           'm', 'LineWidth', 2, 'DisplayName','Trail');
    % 
    % % Punto di contatto anteriore (per chiarezza)
    % plot3(ax, P.p_contatto_ant(1), P.p_contatto_ant(2), P.p_contatto_ant(3), ...
    %           'bs', 'MarkerSize', 8, 'MarkerFaceColor','b', ...
    %           'DisplayName','Punto contatto ant');
    % 
    % % Aggiorna la struttura P con i risultati del trail
    % P.p_intersezione_sterzo = p_intersect;
    % P.Trail_value = trail;
    % P.NormalTrail_value = NormalTrail;


    %% === Baricentro ===
    % Baricentro totale (rosso-bianco)
    drawCOG(ax, P.p_COG_tot, 20, 'r');

    %% === Trasmissione: Corona, Pignone e Catena (tutto nero, X-Z plane) ===
    theta = linspace(0,2*pi,100);
    
    % --- Corona ---
    r_sprocket = (data.Transmission.FinalSprocket * data.Transmission.FinalChainModule) / (2*pi);
    circle_local = [r_sprocket*cos(theta);
                    zeros(1,length(theta));
                    r_sprocket*sin(theta);
                    ones(1,length(theta))];   % punti locali (X-Z plane)
    
    circle_global = T.T_sprocket * circle_local;   % trasformazione nel mondo
    plot3(ax, circle_global(1,:), circle_global(2,:), circle_global(3,:), '-k','LineWidth',1.5);
    scatter3(ax, T.T_sprocket(1,4), T.T_sprocket(2,4), T.T_sprocket(3,4), 30, 'k','filled');
    
    % --- Pignone ---
    r_pinion = (data.Transmission.FinalPinion * data.Transmission.FinalChainModule) / (2*pi);
    circle_local = [r_pinion*cos(theta);
                    zeros(1,length(theta));
                    r_pinion*sin(theta);
                    ones(1,length(theta))]; 
    
    circle_global = T.T_pinion * circle_local;
    plot3(ax, circle_global(1,:), circle_global(2,:), circle_global(3,:), '-k','LineWidth',1.5);
    scatter3(ax, T.T_pinion(1,4), T.T_pinion(2,4), T.T_pinion(3,4), 30, 'k','filled');
    
    % --- Catena superiore ---
    plot3(ax, [P.p_sup_sprocket(1), P.p_sup_pinion(1)], ...
              [P.p_sup_sprocket(2), P.p_sup_pinion(2)], ...
              [P.p_sup_sprocket(3), P.p_sup_pinion(3)], ...
              '-k', 'LineWidth', 2);
    
    % Punti di tangenza
    scatter3(ax, P.p_sup_sprocket(1), P.p_sup_sprocket(2), P.p_sup_sprocket(3), 25, 'k','filled');
    scatter3(ax, P.p_sup_pinion(1), P.p_sup_pinion(2), P.p_sup_pinion(3), 25, 'k','filled');

    %% === Etichette assi ===
    title('Motorcycle Multibody Visualization');
    xlabel('X [mm]');
    ylabel('Y [mm]');
    zlabel('Z [mm]');
end
