function plotBikeSymmetryPlane(P, data, Output)
% plotBikeSymmetryPlane - Disegna la moto in 2D nel piano di simmetria
%
% Questa funzione visualizza la geometria della moto (in forma semplificata)
% nel piano di simmetria, includendo telaio, ruote, trasmissione, baricentro
% e le linee cinematiche (Squat Line e Transfer Line).
%
% INPUT:
%   P      : struct con i punti della moto già proiettati nel piano di simmetria
%            --> punti accessibili come P.bikePlane.p_nome
%   data   : struct con parametri geometrici (ruote, trasmissione ecc.)
%   Output : struct contenente i punti geometrici derivati (es. squat, transfer)
%
% OUTPUT:
%   Un plot 2D (X-Z) della moto nel suo piano di simmetria.


    %% === Profilo telaio ===
    % (non includo i punti di contatto nella linea nera)
    X = [P.bikePlane.p_centro_post(1), P.bikePlane.p_pivot(1), ...
         P.bikePlane.p_piastra_sup_dx(1), ...
         P.bikePlane.p_centro_ant(1)];
    Z = [P.bikePlane.p_centro_post(3), P.bikePlane.p_pivot(3), ...
         P.bikePlane.p_piastra_sup_dx(3), ...
         P.bikePlane.p_centro_ant(3)];
    figure('Position', [100, 100, 1600, 900]);
    plot(X, Z, '-b','LineWidth',2,'MarkerFaceColor','k'); hold on;
    xlim([-500 1700]); ylim([-200 1000]); grid minor; grid on;
    % Punti di contatto evidenziati in rosso
    scatter(P.bikePlane.p_contatto_post(1), P.bikePlane.p_contatto_post(3), ...
            80, 'ro', 'filled','DisplayName','Contatto Post');
    scatter(P.bikePlane.p_contatto_ant(1), P.bikePlane.p_contatto_ant(3), ...
            80, 'ro', 'filled','DisplayName','Contatto Ant');

    %% === Strada ===
    plot([-500, 1700], [0 0], 'k', 'LineWidth', 1.5, 'DisplayName','Strada');


    %% === Ruote ===
    theta = linspace(0,2*pi,200);

    % Ruota posteriore
    x_rear = P.bikePlane.p_centro_post(1) + P.bikePlane.p_centro_post(3) * cos(theta);
    z_rear = P.bikePlane.p_centro_post(3) + P.bikePlane.p_centro_post(3) * sin(theta);
    plot(x_rear, z_rear, 'k','LineWidth',1.2);

    % Ruota anteriore
    x_front = P.bikePlane.p_centro_ant(1) + P.bikePlane.p_centro_ant(3) * cos(theta);
    z_front = P.bikePlane.p_centro_ant(3) + P.bikePlane.p_centro_ant(3) * sin(theta);
    plot(x_front, z_front, 'k','LineWidth',1.2);

    %% === Trasmissione ===
    theta = linspace(0,2*pi,200);

    % Corona (sprocket) -> assunta centrata sul centro ruota posteriore
    r_sprocket = (data.Transmission.FinalSprocket * data.Transmission.FinalChainModule) / (2*pi);
    x_sprocket = P.bikePlane.p_centro_post(1) + r_sprocket*cos(theta);
    z_sprocket = P.bikePlane.p_centro_post(3) + r_sprocket*sin(theta);
    plot(x_sprocket, z_sprocket, 'k','LineWidth',1.2);

    % Pignone
    r_pinion = (data.Transmission.FinalPinion * data.Transmission.FinalChainModule) / (2*pi);
    x_pinion = P.bikePlane.p_pinion(1) + r_pinion*cos(theta);
    z_pinion = P.bikePlane.p_pinion(3) + r_pinion*sin(theta);
    plot(x_pinion, z_pinion, 'k','LineWidth',1.2);

    % Catena superiore (linea tra i punti di tangenza)
    plot([P.bikePlane.p_sup_sprocket(1), P.bikePlane.p_sup_pinion(1)], ...
         [P.bikePlane.p_sup_sprocket(3), P.bikePlane.p_sup_pinion(3)], ...
         '-k','LineWidth',1.2);

    %% === Baricentro (cerchio diviso in 4 quadranti curvi) ===
    r = 20;  % raggio del cerchio del COG
    centerX = P.bikePlane.p_COG_tot(1);  % coordinata X del COG
    centerZ = P.bikePlane.p_COG_tot(3);  % coordinata Z del COG
    
    theta_edges = linspace(0, 2*pi, 5);  % divisione in 4 quadranti
    colors = {'b','w'};                   % alternanza colori quadranti
    
    hold on;
    for i = 1:4
        t = linspace(theta_edges(i), theta_edges(i+1), 50);
        x_patch = [centerX, centerX + r*cos(t), centerX];
        z_patch = [centerZ, centerZ + r*sin(t), centerZ];
        
        patch(x_patch, z_patch, colors{mod(i,2)+1}, 'EdgeColor', 'none'); % quadrante
    end
    
    % Bordo esterno del cerchio baricentro
    theta = linspace(0, 2*pi, 200);
    x_circle = centerX + r*cos(theta);
    z_circle = centerZ + r*sin(theta);
    plot(x_circle, z_circle, 'k', 'LineWidth', 1.5);


    %% === Punto di Intersezione (stella) ===
    scatter(P.bikePlane.p_intersect(1), P.bikePlane.p_intersect(3), ...
        40, 'o','MarkerEdgeColor','r','MarkerFaceColor','r', ...
        'DisplayName','Intersect');

    %% === Linee cinematiche ===

    L = 1500;  % lunghezza fittizia delle linee da plottare
    
    % Punto di partenza (contatto posteriore)
    x0 = P.bikePlane.p_contatto_post(1);
    z0 = P.bikePlane.p_contatto_post(3);
    
    % Squat Line
    x_squat = [x0, x0 + L * cosd(Output.SquatAngle)];
    z_squat = [z0, z0 + L * sind(Output.SquatAngle)];
    plot(x_squat, z_squat, '-.m', 'LineWidth', 2, 'DisplayName', 'Squat Line');
    
    % Transfer Line
    x_transfer = [x0, x0 + L * cosd(Output.TransferAngle)];
    z_transfer = [z0, z0 + L * sind(Output.TransferAngle)];
    plot(x_transfer, z_transfer, '-.c', 'LineWidth', 2, 'DisplayName', 'Transfer Line');

    %% === Archi degli angoli ===
    % Centro degli archi = punto di contatto posteriore
    x0 = P.bikePlane.p_contatto_post(1);
    z0 = P.bikePlane.p_contatto_post(3);
    r_arc = 200; % raggio fittizio per gli archi
    r_arc_2 = 400; % raggio fittizio per gli archi
    
    % Squat Angle (arco da 0 a SquatAngle)
    theta_squat = linspace(0, deg2rad(Output.SquatAngle), 100);
    xs = x0 + r_arc * cos(theta_squat);
    zs = z0 + r_arc * sin(theta_squat);
    plot(xs, zs, '-m','LineWidth',2);
    text(xs(end)+20, zs(end), 'Squat', 'Color','m','FontWeight','bold');

    % Transfer Angle (arco da 0 a TransferAngle)
    theta_transfer = linspace(0, deg2rad(Output.TransferAngle), 100);
    xt = x0 + r_arc_2 * cos(theta_transfer);
    zt = z0 + r_arc_2 * sin(theta_transfer);
    plot(xt, zt, '-c','LineWidth',2);
    text(xt(end)+20, zt(end), 'Transfer', 'Color','c','FontWeight','bold');



    % legend('Location','Best');
end
