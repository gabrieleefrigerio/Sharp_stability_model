function [Output, P] = OutputMultibody(T, P, data,flag_speed)
% OutputMultibody - Calcola i principali parametri cinematici e geometrici del modello multibody della moto.
%
% Author: Antonio Filianoti
%
% DESCRIPTION:
%   Questa funzione riceve in input le trasformazioni omogenee (T), i punti chiave nello spazio (P),
%   e i parametri del veicolo (data), e restituisce una struttura 'Output' contenente
%   le quantità geometriche e cinematiche della moto come:
%     - Wheelbase
%     - Caster angle
%     - Trail
%     - Squat ratio
%     - Altezza baricentro
%     - Interasse sterzo-forcellone, ecc.
%
% INPUT:
%   T      - Struct contenente le matrici di trasformazione omogenea 4x4 per ogni frame del sistema
%   P      - Struct contenente le coordinate 3D di tutti i punti chiave (centri ruote, pivot, ecc.)
%   data   - Struct contenente i parametri geometrici/configurazione della moto (sospensioni, angoli, etc.)
%
% OUTPUT:
%   Output - Struct contenente i parametri cinematici e geometrici calcolati del modello multibody
%            (wheelbase, caster angle, trail, squat ratio, etc.)
%
% USO:
%   Output = OutputMultibody(T, P, data);
%

% inizializzo la struct degli output
Output = struct();

% Calcola il passo come distanza orizzontale (sul piano XZ)
if data.Multibody.Roll == 0 && data.Multibody.SteeringAngle == 0
    delta = P.p_centro_ant - P.p_centro_post;
else
    delta = P.p_contatto_ant - P.p_contatto_post;
end

Output.Wheelbase = round(norm([delta(1), delta(2), delta(3)]),1);  % usa solo X e Z

% trovo l'angolo di sterzo
Output.caster_angle = computeCasterAngle(T.T_steering_axis, T.T_SDR_abs);

% trovo l'avancorsa geometrica e quella normale
[Output.Trail, Output.NormalTrail, P.Trail, ~] = trailComputation(T, P, Output);

% trovo l'angolo di sterzata effettivo sul piano xy (considerando il
% sideslip nullo
Output.EffectiveSteeringAngle = EffectiveSteeringAngle(T);

% salvo l'angolo del forcellone
Output.SwingarmAngle = rad2deg(data.Multibody.AlphaSwingArm);

% salvo la chain elongation

if nargin == 3
    [Output.ChainElong,~] = chain_elong(data);
end

%cog
Output.COG = round(P.p_COG_tot([1:3],1));


%distr pesi 
Output.distr = [round(Output.COG(1)/Output.Wheelbase*100,1), round((1-Output.COG(1)/Output.Wheelbase)*100,1)];

% calcolo squat ratio e squat angle
[Output.SquatAngle, Output.TransferAngle , Output.SquatRatio, P] = compute_squat_ratio(P);

% calcolo raggio effettivo gomme
Output.RearEffectiveRadius = round(P.bikePlane.p_centro_post(3),1);
Output.FrontEffectiveRadius = round(P.bikePlane.p_centro_ant(3),1);


% arrotondo i valori
Output.Trail = round(Output.Trail,1);
Output.NormalTrail = round(Output.NormalTrail,1);
if nargin == 3
    Output.ChainElong = round(Output.ChainElong,2);
end
Output.SwingarmAngle = abs(round(Output.SwingarmAngle,1));
Output.EffectiveSteeringAngle = round(Output.EffectiveSteeringAngle,1);
Output.caster_angle = round(Output.caster_angle,1);
Output.SquatAngle = round(Output.SquatAngle,1);
Output.SquatRatio = round(Output.SquatRatio,3);
Output.TransferAngle = round(Output.TransferAngle,1);

end


