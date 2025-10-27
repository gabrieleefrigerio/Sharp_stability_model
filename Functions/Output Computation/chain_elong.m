function [chain_elong,compr] = chain_elong(data) %DA CAMBIARE NOME
% TIRO CATENA .
%
% INPUT:
%   data - struct contenente le informazioni geometriche e cinematiche
%
% OUTPUT:
%  
%   chain_elong  -  [mm] escusione massima lunghezza
%   interasse corona pignone
%   compr - compressione mono e forche a cui ho distanza interasse minima
%   [mm] [forche,mono]

% Author: Nicola Cabrini

%inizializzo vettore lunghezza interasse
Rear_Susp_Compr = linspace(0,data.RearSuspension.MaxCompressionRearSuspension,data.RearSuspension.MaxCompressionRearSuspension);
l_interasse = zeros(1,length(Rear_Susp_Compr));

for jj = 1:length(Rear_Susp_Compr)
    % assegno compressione che varia
    data.RearSuspension.Compression = Rear_Susp_Compr(jj);

    % chiusure cinamatiche per calcolo angolo forcellone e angolo sdr telaio 
    [data,P] = cinematicClosures(data);

    deltax = (cos(-data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal + P.Coordinate2D.Pinion(1)*cos(data.Multibody.BetaSDRFrame) - P.Coordinate2D.Pinion(2)*sin(data.Multibody.BetaSDRFrame));
    deltay = (sin(-data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal + P.Coordinate2D.Pinion(1)*sin(data.Multibody.BetaSDRFrame) + P.Coordinate2D.Pinion(2)*cos(data.Multibody.BetaSDRFrame));
    l_interasse(1,jj) = sqrt(deltax^2+deltay^2);
end

[MAX,IMAX] = max(l_interasse);
[MIN,~] = min(l_interasse);
chain_elong = MAX - MIN;
[compr] = Rear_Susp_Compr(IMAX);

%% versione con variazione anche di front susp
% %inizializzo vettore lunghezza interasse
% Rear_Susp_Compr = linspace(0,data.RearSuspension.MaxCompressionRearSuspension,2*data.RearSuspension.MaxCompressionRearSuspension);
% Front_Susp_Compr = linspace(0,data.FrontSuspension.ForksMaxCompression,data.FrontSuspension.ForksMaxCompression/2);
% l_interasse = zeros(length(Front_Susp_Compr),length(Rear_Susp_Compr));
% 
% for ii = 1: length(Front_Susp_Compr)
%     data.FrontSuspension.Compression = Front_Susp_Compr(ii);
%     for jj = 1:length(Rear_Susp_Compr)
%         % assegno compressione che varia
%         data.RearSuspension.Compression = Rear_Susp_Compr(jj);
% 
%         % chiusure cinamatiche per calcolo angolo forcellone e angolo sdr telaio 
%         [data,P] = cinematicClosures(data);
% 
%         deltax = (cos(-data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal + P.Coordinate2D.Pinion(1)*cos(data.Multibody.BetaSDRFrame) - P.Coordinate2D.Pinion(2)*sin(data.Multibody.BetaSDRFrame));
%         deltay = (sin(-data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal + P.Coordinate2D.Pinion(1)*sin(data.Multibody.BetaSDRFrame) + P.Coordinate2D.Pinion(2)*cos(data.Multibody.BetaSDRFrame));
%         l_interasse(ii,jj) = sqrt(deltax^2+deltay^2);
%     end
% end
% [MAX,IMAX] = max(l_interasse,[],"all","linear");
% [MIN,~] = min(l_interasse,[],"all","linear");
% chain_elong = MAX - MIN;
% 
% [index1, index2] = ind2sub(size(l_interasse),IMAX);
% compr(1) = Front_Susp_Compr(index1)
% compr(2) = Rear_Susp_Compr(index2)




end


