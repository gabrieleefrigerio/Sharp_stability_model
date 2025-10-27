function [Coordinate,data] = computeCoordinates(data)
% COMPUTECOORDINATES Calcola le coordinate globali dei punti con angolo e offset.
%
% INPUT:
%   data - struct contenente le informazioni geometriche e cinematiche
%
% OUTPUT:
%   Coordinate - struct con le posizioni globali dei punti trasformati 

% Author: Antonio Filianoti


    Coordinate = struct(); % Inizializza la struct di output

    

    %% === FRAME ===
    % Coordinate Attacco Telaio Sup
    origin = [data.Frame.AttFrameSupX; data.Frame.AttFrameSupZ];
    angle = data.Frame.AngleAttFrameSupOffset;
    offset = [data.Frame.AttFrameSupOffset; 0];
    Coordinate.AttFrameSup = rotoTrasla2D(origin, angle, offset);

    % Coordinate Attacco Telaio Inf
    origin = [data.Frame.AttFrameInfX; data.Frame.AttFrameInfZ];
    angle = data.Frame.AngleAttFrameInfOffset;
    offset = [data.Frame.AttFrameInfOffset; 0];
    Coordinate.AttFrameInf = rotoTrasla2D(origin, angle, offset);

    % Pivot: origine è [0;0], offset è direttamente [X; Z]
    origin = [0; 0];
    angle = - data.Frame.AnglePivotOffset;
    offset = [data.Frame.PivotOffsetX; data.Frame.PivotOffsetZ];
    Coordinate.Pivot = rotoTrasla2D(origin, angle, offset);

    % Canotto
    Coordinate.l_frame = data.Frame.LengthRear2SteeringHead - Coordinate.Pivot(1) + data.Frame.SteeringAxisTopOffset;                                 % lunghezza telaio effettiva da pivot a canotto (asse z telaio)
    Coordinate.h_frame = data.Frame.HeightSteeringHead - Coordinate.Pivot(2) ;   % altezza effettiva telaio pivot canotto (asse x telaio)

    % Trovo le coordinate dei nuovi punti rispetto al nuovo pivot
    Coordinate.AttFrameSup = Coordinate.AttFrameSup - Coordinate.Pivot;
    Coordinate.AttFrameInf = Coordinate.AttFrameInf - Coordinate.Pivot;
    data.Frame.COGX = data.Frame.COGX - Coordinate.Pivot(1);
    data.Frame.COGZ = data.Frame.COGZ - Coordinate.Pivot(2);
    Coordinate.Pinion = [data.Frame.PinionX; data.Frame.PinionZ] - Coordinate.Pivot;
    Coordinate.Pivot =  origin;

    % lunghezza effettiva telaio
    data.Multibody.LengthFrameFinal = sqrt(Coordinate.h_frame^2 + Coordinate.l_frame^2);
    

    %% === SWINGARM ===
    % Attacco Forcellone Inf
    origin = [data.Swingarm.AttSwingInfX; data.Swingarm.AttSwingInfZ];
    angle = data.Swingarm.AngleAttSwingInf;
    offset = [data.Swingarm.OffsetAttSwingInf; 0];
    Coordinate.AttSwingInf = rotoTrasla2D(origin, angle, offset);

    % Attacco Forcellone Sup
    origin = [data.Swingarm.AttSwingSupX; data.Swingarm.AttSwingSupZ];
    angle = data.Swingarm.AngleAttSwingSup;
    offset = [data.Swingarm.OffsetAttSwingSup; 0];
    Coordinate.AttSwingSup = rotoTrasla2D(origin, angle, offset);

    % Calcolo della lunghezza equivalente con il teorema del coseno
    l1 = data.Swingarm.Length;
    l2 = data.Swingarm.Offset;
    theta = data.Swingarm.AngleOffset; % angolo tra i due lati

    % Lunghezza risultante tra i due punti
    data.Multibody.LengthSwingArmFinal = sqrt(l1^2 + l2^2 + 2 * l1 * l2 * cos(theta));
    

end


