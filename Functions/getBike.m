function [Bike, Output, P, T] = getBike(Bike)
% =========================================================================
% getBike - Imposta i parametri iniziali della motocicletta e calcola
%           le matrici cinematiche e multibody associate.
%
% DESCRIZIONE:
%   Questa funzione inizializza gli angoli principali della moto (rollio,
%   sterzo, pendenza, banking), aggiorna la cinematica del sistema e
%   calcola le matrici dinamiche associate. Infine determina l’offset
%   geometrico del gruppo anteriore rispetto all’asse di sterzo.
%
% INPUT:
%   Bike  -> struttura dati contenente parametri geometrici e dinamici
%
% OUTPUT:
%   Bike  -> struttura aggiornata con nuove variabili e offset geometrico
%   Output -> risultati del modello multibody (output cinematici/dinamici)
%   P      -> struttura con punti e parametri cinematici
%   T      -> struttura con matrici di trasformazione
%
% FUNZIONI RICHIAMATE:
%   cinematicClosures()   - Calcola le chiusure cinematiche del modello
%   MultibodyMatrices()   - Costruisce le matrici multibody T e P
%   OutputMultibody()     - Calcola le uscite cinematiche del modello
%   calcOffsetFromHT()    - Determina l’offset geometrico dal gruppo anteriore
%
% ==========================================================================

    % ---------------------- Parametri stradali ----------------------
    % Pendenza (Road Slope) e banking della strada (angolo laterale)
    Bike.Road.RoadSlope = deg2rad(0);   % [rad] inclinazione longitudinale strada
    Bike.Road.Banking   = deg2rad(0);   % [rad] inclinazione laterale strada

    % ---------------------- Stato multibody iniziale ----------------------
    Bike.Multibody.Roll          = deg2rad(0);  % [rad] angolo di rollio iniziale
    Bike.Multibody.SteeringAngle = deg2rad(0);  % [rad] angolo di sterzo iniziale

    % ---------------------- Stato sospensioni ----------------------
    data.FrontSuspension.Compression = 0;  % [m] compressione sospensione anteriore
    data.RearSuspension.Compression  = 0;  % [m] compressione sospensione posteriore

    % ---------------------- Chiusure cinematiche ----------------------
    % Calcola le variabili cinematiche dei punti del modello (P)
    [Bike, P] = cinematicClosures(Bike);

    % ---------------------- Matrici multibody ----------------------
    % Calcola le matrici di trasformazione (T) e aggiorna P e Bike
    [T, P, Bike] = MultibodyMatrices(Bike, P);

    % ---------------------- Output multibody ----------------------
    % Calcola le quantità cinematiche e dinamiche di interesse (Output)
    [Output, P] = OutputMultibody(T, P, Bike);

    % ---------------------- Calcolo offset geometrico ----------------------
    % Ottiene l'offset e tra asse sterzo e baricentro gruppo anteriore
    G = P.COG_front_assembl;    % posizione baricentro gruppo anteriore
    H = T.T_steering_axis;      % trasformazione asse di sterzo
    offset_e = calcOffsetFromHT(H, G);

    % Salva l'offset nella struttura principale Bike
    Bike.offset_e = offset_e;

end
