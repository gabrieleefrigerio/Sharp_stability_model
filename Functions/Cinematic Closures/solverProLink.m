function [data,Coordinate] = solverProLink(data, Coordinate)
% solverProLink Risolve le chiusure cinematiche della moto con schema
% prolink
% e trova angolo forcellone finale, passo, e angolo di sterzo
% e angolo inclinazione mono
%
% INPUT:
%   data - struct contenente le informazioni geometriche e cinematiche
%   Coordinate - struct con le posizioni globali dei punti trasformati
%
% OUTPUT: data - struct contenente le informazioni geometriche e cinematiche

% Author: NicolaCabrini
%% vers 1 usando coordinate e avendo punti in sdr telaio rispetto nuovo pivot
%calcolo moduli e angoli degli attacchi telaio e forcellone (post regolazioni) rispetto ai relativi sdr 
AttSwingInfMod = sqrt(Coordinate.AttSwingInf(1)^2+Coordinate.AttSwingInf(2)^2); 
AttSwingInfAng = aatan(Coordinate.AttSwingInf(2),Coordinate.AttSwingInf(1));
AttFrameInfMod = sqrt(Coordinate.AttFrameInf(1)^2+Coordinate.AttFrameInf(2)^2);
AttFrameInfAng = aatan(Coordinate.AttFrameInf(2),Coordinate.AttFrameInf(1));
AttFrameSupMod = sqrt(Coordinate.AttFrameSup(1)^2+Coordinate.AttFrameSup(2)^2);
AttFrameSupAng = aatan(Coordinate.AttFrameSup(2),Coordinate.AttFrameSup(1));

%calcolo angolo di inclinazione Lframefinal rispetto a sdr frame 
% e angolo compreso tra - x_sdr_swing e il Lswingarmfinal
AngleFrameFinal = atan(Coordinate.h_frame/Coordinate.l_frame);
l1 = data.Swingarm.Length;
l2 = data.Swingarm.Offset;
AngleSwingArmFinal = acos((l2^2-l1^2-data.Multibody.LengthSwingArmFinal^2)/(-2*l1*data.Multibody.LengthSwingArmFinal));

%angolo rocker opposto a FrameSusp
angrock = acos((+data.RearSuspension.TriangleRodSusp^2 - data.RearSuspension.TriangleSwingRod^2 - data.RearSuspension.TriangleSwingSusp^2)/(-2*data.RearSuspension.TriangleSwingSusp*data.RearSuspension.TriangleSwingRod)); 
%calcolo gamma (inclinazione sdr forche rispetto a sdr telaio)
gamma = -pi/2 + aatan(data.Frame.DistSteeringHead,data.Frame.SteeringAxisTopOffset - data.Frame.SteeringAxisBottomOffset);  

% correzione angolo swingarm finale in base a segno di angolo regolazione
% asola
if data.Swingarm.AngleOffset>=0 && data.Swingarm.AngleOffset<=180
    angswing = @(a) a-AngleSwingArmFinal;
else
    angswing = @(a) a+AngleSwingArmFinal;
end

% angoli tutti rispetto a sdr totale 
% x = [angolo sdr swingarm, angolo sdr telaio, angolo lato swingrod,...
% angolo biella, angolo inclinazione mono, passo moto]
fun1 = @(x) [AttSwingInfMod*cos(AttSwingInfAng+x(1))+data.RearSuspension.TriangleSwingRod*cos(x(3))+data.RearSuspension.RodLength*cos(x(4)) - AttFrameInfMod*cos(AttFrameInfAng+x(2));
             AttSwingInfMod*sin(AttSwingInfAng+x(1))+data.RearSuspension.TriangleSwingRod*sin(x(3))+data.RearSuspension.RodLength*sin(x(4)) - AttFrameInfMod*sin(AttFrameInfAng+x(2));
             AttSwingInfMod*cos(AttSwingInfAng+x(1))+data.RearSuspension.TriangleSwingSusp*cos(x(3)+angrock)+ (data.RearSuspension.LengthRearSuspension-data.RearSuspension.Compression)*cos(x(5)) - AttFrameSupMod*cos(AttFrameSupAng+x(2));
             AttSwingInfMod*sin(AttSwingInfAng+x(1))+data.RearSuspension.TriangleSwingSusp*sin(x(3)+angrock)+ (data.RearSuspension.LengthRearSuspension-data.RearSuspension.Compression)*sin(x(5)) - AttFrameSupMod*sin(AttFrameSupAng+x(2));
             data.Wheels.RearSize*cos(pi/2)+data.Multibody.LengthSwingArmFinal*cos(angswing(x(1)))+data.Multibody.LengthFrameFinal*cos(x(2)+AngleFrameFinal)+(data.FrontSuspension.OffsetTop+data.FrontSuspension.PlatesForkOffset)*cos(gamma+x(2))+data.FrontSuspension.PlatesTopThickness*cos(gamma+x(2)+(pi/2))+(data.FrontSuspension.ForksLength-data.FrontSuspension.ForksVerticalOffset-data.FrontSuspension.Compression)*cos(gamma+x(2)+((3/2)*pi))+data.FrontSuspension.FootOffset*cos(gamma+x(2))+data.Wheels.FrontSize*cos(((3/2)*pi))+x(6)*cos(pi);
             data.Wheels.RearSize*sin(pi/2)+data.Multibody.LengthSwingArmFinal*sin(angswing(x(1)))+data.Multibody.LengthFrameFinal*sin(x(2)+AngleFrameFinal)+(data.FrontSuspension.OffsetTop+data.FrontSuspension.PlatesForkOffset)*sin(gamma+x(2))+data.FrontSuspension.PlatesTopThickness*sin(gamma+x(2)+(pi/2))+(data.FrontSuspension.ForksLength-data.FrontSuspension.ForksVerticalOffset-data.FrontSuspension.Compression)*sin(gamma+x(2)+((3/2)*pi))+data.FrontSuspension.FootOffset*sin(gamma+x(2))+data.Wheels.FrontSize*sin(((3/2)*pi))+x(6)*sin(pi)];

%risolvo sistema
options = optimoptions('fsolve','Display','off');
xsol1 = fsolve(fun1,[0.3142 0.4014 0 0.8727 1.5708 1225],options);

% %plot per controllo da togliere
% beta1 = rad2deg(AngleFrameFinal+xsol1(2))
% alpha1 = rad2deg(angswing(xsol1(1)))
% rake1 = rad2deg(xsol1(2)+gamma)
% passo1 = xsol1(6)

% %salvo soluzioni SOLO QUANDO FUNZIONERà
% data.Multibody.AlphaSwingArm       = -angswing(xsol1(1));
% data.Multibody.GammaSteeringAxis   = xsol1(2)+gamma;
% data.Multibody.BetaFrame           = -deg2rad(AngleFrameFinal+xsol1(2));

data.Multibody.AlphaSwingArm       = -angswing(xsol1(1));
%data.Multibody.GammaSteeringAxis   = xsol1(2)+gamma;
data.Multibody.Gamma   = gamma;
data.Multibody.BetaSDRFrame           = xsol1(2);  %per salvataggio con sdr telaio vero
% %Passo e angolo mono non so se vanno salvati da qualche parte

%% vers 2 senza coordinate. chiusure con dentro tutte le regolazioni ancora da finire
% angrock = acos((-data.RearSuspension.TriangleRodSusp^2 + data.RearSuspension.TriangleSuspFrame^2 - data.RearSuspension.TriangleFrameRod^2)/(-2*data.RearSuspension.TriangleFrameRod*data.RearSuspension.TriangleRodSusp));%angolo rocker opposto a FrameSusp
% %calcolo gamma (inclinazione sdr forche rispetto a sdr telaio)
% gamma = -pi/2 + aatan(data.Frame.DistSteeringHead,data.Frame.SteeringAxisTopOffset - data.Frame.SteeringAxisBottomOffset  );
% 
% % angoli tutti rispetto a sdr totale 
% % x = [angolo sdr swingarm, angolo sdr telaio, angolo biella,...
% % angolo lato RodFrame rocker, angolo inclinazione mono, passo moto]
% fun2 = @(x) %da scrivere
% %risolvo sistema
% options = optimoptions('fsolve','Display','off');
% xsol2 = fsolve(fun2,[0.3142 0.4014 0 0.8727 1.5708 1225],options);
% 
% % %plot per controllo da togliere
% % beta2 = rad2deg(AngleFrameFinal+xsol2(2))
% % alpha2 = rad2deg(xsol2(1))
% % rake2 = rad2deg(xsol2(2)+gamma)
% % passo2 = xsol2(6)

% %salvo soluzioni 
% data.Multibody.AlphaSwingArm       = angswing(xsol2(1));
% data.Multibody.GammaSteeringAxis   = xsol2(2)+gamma;
% data.Multibody.BetaFrame           = deg2rad(AngleFrameFinal+xsol2(2));
% % %Passo e angolo mono non so se vanno salvati da qualche parte

%%  %funzione per calcolare arcotangenta anche in secondo e terzo quadrante
    function out = aatan(opp,adia) %opp z adia x
        if opp>=0 && adia>0
            out = atan(opp/adia);
        elseif opp<=0 && adia>0
            out = atan(opp/adia);
        elseif opp<=0 && adia<0
            out = pi + atan(opp/adia);
        elseif opp>=0 && adia<0
            out = pi + atan(opp/adia);
        elseif adia==0
            out = pi/2;
        end
    end
end



