function [data,P] = cinematicClosures(data)
% CINEMATICCLOSURES Calcola le chiusure cinematiche di un sistema moto
%
% Questa funzione prende in input una struct `Bike` contenente le informazioni
% geometriche e cinematiche del veicolo (telaio, sospensioni, etc.) e calcola 
% le chiusure cinematiche tra i vari corpi rigidi, tipicamente per determinare 
% la consistenza geometrica e la posizione relativa dei componenti.
%
% OUTPUT:
%   Output - Struct contenente i risultati delle chiusure cinematiche, 
%            come errori di chiusura o coordinate calcolate.

% Author: Antonio Filianoti


% richiamo la funzione per trovare le coordinate e le lunghezze equivalenti
% di telaio e forcellone in base alle regolazioni da usare per le chiusure
% nel piano
[Coordinate2D,data] = computeCoordinates(data);

% in dependence of the cinematic scheme call a specific sovler function
if data.RearSuspension.Type == 2
    [data,Coordinate2D] = solverProLink(data, Coordinate2D);
elseif data.RearSuspension.Type == 4
    [data,Coordinate2D] = solverUnitrack(data, Coordinate2D);
end

P.Coordinate2D = Coordinate2D;
end


