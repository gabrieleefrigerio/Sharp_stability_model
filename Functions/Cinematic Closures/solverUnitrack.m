function [data,Coordinate] = solverUnitrack(data, Coordinate)
% solverUnitrack Risolve le chiusure cinematiche della moto 
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
angrock = acos((-data.RearSuspension.TriangleRodSusp^2 + data.RearSuspension.TriangleSuspFrame^2 - data.RearSuspension.TriangleFrameRod^2)/(-2*data.RearSuspension.TriangleFrameRod*data.RearSuspension.TriangleRodSusp)); 
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
% x = [angolo sdr swingarm, angolo sdr telaio, angolo biella,...
% angolo lato RodFrame rocker, angolo inclinazione mono, passo moto]
fun1 = @(x) [AttSwingInfMod*cos(AttSwingInfAng+x(1))+data.RearSuspension.RodLength*cos(x(3))+data.RearSuspension.TriangleFrameRod*cos(x(4)) - AttFrameInfMod*cos(AttFrameInfAng+x(2));
             AttSwingInfMod*sin(AttSwingInfAng+x(1))+data.RearSuspension.RodLength*sin(x(3))+data.RearSuspension.TriangleFrameRod*sin(x(4)) - AttFrameInfMod*sin(AttFrameInfAng+x(2));
             AttSwingInfMod*cos(AttSwingInfAng+x(1))+data.RearSuspension.RodLength*cos(x(3))+data.RearSuspension.TriangleRodSusp*cos(x(4)+angrock) + (data.RearSuspension.LengthRearSuspension-data.RearSuspension.Compression)*cos(x(5)) - AttFrameSupMod*cos(AttFrameSupAng+x(2));
             AttSwingInfMod*sin(AttSwingInfAng+x(1))+data.RearSuspension.RodLength*sin(x(3))+data.RearSuspension.TriangleRodSusp*sin(x(4)+angrock) + (data.RearSuspension.LengthRearSuspension-data.RearSuspension.Compression)*sin(x(5)) - AttFrameSupMod*sin(AttFrameSupAng+x(2));
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

l_forks_final = data.FrontSuspension.ForksLength-data.FrontSuspension.ForksVerticalOffset-data.FrontSuspension.Compression;
Coordinate.p_contatto_post = [0,0];
Coordinate.p_centro_post = [0,data.Wheels.RearSize];
Coordinate.p_contatto_ant = [xsol1(6),0];
Coordinate.p_centro_ant = [xsol1(6),data.Wheels.FrontSize];
Coordinate.p_pivot = Coordinate.p_centro_post + [data.Multibody.LengthSwingArmFinal*cos(angswing(xsol1(1))),data.Multibody.LengthSwingArmFinal*sin(angswing(xsol1(1)))];
Coordinate.p_steering_axis = Coordinate.p_pivot + [data.Multibody.LengthFrameFinal*cos(AngleFrameFinal+xsol1(2)),data.Multibody.LengthFrameFinal*sin(AngleFrameFinal+xsol1(2))];
Coordinate.p_piastra_sup = Coordinate.p_centro_ant +[-l_forks_final*sin(xsol1(2)+gamma),l_forks_final*cos(xsol1(2)+gamma)];

%% vers 2 senza coordinate. chiusure con dentro tutte le regolazioni
% angrock = acos((-data.RearSuspension.TriangleRodSusp^2 + data.RearSuspension.TriangleSuspFrame^2 - data.RearSuspension.TriangleFrameRod^2)/(-2*data.RearSuspension.TriangleFrameRod*data.RearSuspension.TriangleRodSusp));%angolo rocker opposto a FrameSusp
% %calcolo gamma (inclinazione sdr forche rispetto a sdr telaio)
% gamma = -pi/2 + aatan(data.Frame.DistSteeringHead,data.Frame.SteeringAxisTopOffset - data.Frame.SteeringAxisBottomOffset  );
% 
% % angoli tutti rispetto a sdr totale 
% % x = [angolo sdr swingarm, angolo sdr telaio, angolo biella,...
% % angolo lato RodFrame rocker, angolo inclinazione mono, passo moto]
% fun2 = @(x) [data.Swingarm.AttSwingInfX*cos((x(1)))+data.Swingarm.AttSwingInfZ*cos(((1/2)*pi+x(1)))+data.Swingarm.OffsetAttSwingInf*cos((x(1)+data.Swingarm.AngleAttSwingInf))+data.RearSuspension.RodLength*cos((x(3)))+data.RearSuspension.TriangleFrameRod*cos((x(4)))+data.Frame.PivotOffsetX*cos((x(2)-data.Frame.AnglePivotOffset))+data.Frame.PivotOffsetZ*cos((x(2)-data.Frame.AnglePivotOffset+(pi/2)))-(data.Frame.AttFrameInfX*cos((x(2)))+data.Frame.AttFrameInfZ*cos((x(2)+(pi/2)))+data.Frame.AttFrameInfOffset*cos((x(2)+data.Frame.AngleAttFrameInfOffset)));
%             data.Swingarm.AttSwingInfX*cos((x(1)))+data.Swingarm.AttSwingInfZ*cos(((1/2)*pi + x(1)))+data.Swingarm.OffsetAttSwingInf*cos((x(1)+data.Swingarm.AngleAttSwingInf))+data.RearSuspension.RodLength*cos((x(3)))+data.RearSuspension.TriangleRodSusp*cos((x(4)+angrock))+(data.RearSuspension.LengthRearSuspension-data.RearSuspension.Compression)*cos((x(5)))+data.Frame.PivotOffsetX*cos((x(2)-data.Frame.AnglePivotOffset))+data.Frame.PivotOffsetZ*cos((x(2)-data.Frame.AnglePivotOffset+(pi/2)))-(data.Frame.AttFrameSupX*cos((x(2)))+data.Frame.AttFrameSupZ*cos((x(2)+(pi/2)))+data.Frame.AttFrameSupOffset*cos((x(2)+data.Frame.AngleAttFrameSupOffset)));
%             data.Wheels.RearSize*cos((pi/2))-data.Swingarm.Offset*cos((data.Swingarm.AngleOffset+x(1)+pi))+data.Swingarm.Length*cos((x(1)))-data.Frame.PivotOffsetX*cos((x(2)-data.Frame.AnglePivotOffset))-data.Frame.PivotOffsetZ*cos((x(2)+(1/2)*pi-data.Frame.AnglePivotOffset))+data.Frame.LengthRear2SteeringHead*cos((x(2)))+data.Frame.HeightSteeringHead*cos((x(2)+pi/2))+data.Frame.SteeringAxisTopOffset*cos((x(2)))+(data.FrontSuspension.OffsetTop+data.FrontSuspension.PlatesForkOffset)*cos((x(2)+gamma))+data.FrontSuspension.PlatesTopThickness*cos((x(2)+gamma+pi/2))+(data.FrontSuspension.ForksLength-data.FrontSuspension.Compression-data.FrontSuspension.ForksVerticalOffset)*cos((gamma+x(2)+(pi*(3/2))))+data.FrontSuspension.FootOffset*cos((gamma+x(2)))+data.Wheels.FrontSize*cos(((3/2)*pi))+x(6)*cos((-pi));
%             data.Swingarm.AttSwingInfX*sin((x(1)))+data.Swingarm.AttSwingInfZ*sin(((1/2)*pi + x(1)))+data.Swingarm.OffsetAttSwingInf*sin((x(1)+data.Swingarm.AngleAttSwingInf))+data.RearSuspension.RodLength*sin((x(3)))+data.RearSuspension.TriangleFrameRod*sin((x(4)))+data.Frame.PivotOffsetX*sin((x(2)-data.Frame.AnglePivotOffset))+data.Frame.PivotOffsetZ*sin((x(2)-data.Frame.AnglePivotOffset+(pi/2)))-(data.Frame.AttFrameInfX*sin((x(2)))+data.Frame.AttFrameInfZ*sin((x(2)+(pi/2)))+data.Frame.AttFrameInfOffset*sin((x(2)+data.Frame.AngleAttFrameInfOffset)));
%             data.Swingarm.AttSwingInfX*sin((x(1)))+data.Swingarm.AttSwingInfZ*sin(((1/2)*pi + x(1)))+data.Swingarm.OffsetAttSwingInf*sin((x(1)+data.Swingarm.AngleAttSwingInf))+data.RearSuspension.RodLength*sin((x(3)))+data.RearSuspension.TriangleRodSusp*sin((x(4)+angrock))+(data.RearSuspension.LengthRearSuspension-data.RearSuspension.Compression)*sin((x(5)))+data.Frame.PivotOffsetX*sin((x(2)-data.Frame.AnglePivotOffset))+data.Frame.PivotOffsetZ*sin((x(2)-data.Frame.AnglePivotOffset+(pi/2)))-(data.Frame.AttFrameSupX*sin((x(2)))+data.Frame.AttFrameSupZ*sin((x(2)+(pi/2)))+data.Frame.AttFrameSupOffset*sin((x(2)+data.Frame.AngleAttFrameSupOffset)));
%             data.Wheels.RearSize*sin((pi/2))-data.Swingarm.Offset*sin((data.Swingarm.AngleOffset+x(1)+pi))+data.Swingarm.Length*sin((x(1)))-data.Frame.PivotOffsetX*sin((x(2)-data.Frame.AnglePivotOffset))-data.Frame.PivotOffsetZ*sin((x(2)+(1/2)*pi-data.Frame.AnglePivotOffset))+data.Frame.LengthRear2SteeringHead*sin((x(2)))+data.Frame.HeightSteeringHead*sin((x(2)+pi/2))+data.Frame.SteeringAxisTopOffset*sin((x(2)))+(data.FrontSuspension.OffsetTop+data.FrontSuspension.PlatesForkOffset)*sin((x(2)+gamma))+data.FrontSuspension.PlatesTopThickness*sin((x(2)+gamma+pi/2))+(data.FrontSuspension.ForksLength-data.FrontSuspension.Compression-data.FrontSuspension.ForksVerticalOffset)*sin((gamma+x(2)+(pi*(3/2))))+data.FrontSuspension.FootOffset*sin((gamma+x(2)))+data.Wheels.FrontSize*sin(((3/2)*pi))+x(6)*sin((-pi))];
% 
% %risolvo sistema
% options = optimoptions('fsolve','Display','off');
% xsol2 = fsolve(fun2,[0.3142 0.4014 0 0.8727 1.5708 1225],options);
% 
% % %plot per controllo da togliere
% % beta2 = rad2deg(AngleFrameFinal+xsol2(2))
% % alpha2 = rad2deg(xsol2(1))
% % rake2 = rad2deg(xsol2(2)+gamma)
% % passo2 = xsol2(6)
% 
% %salvo soluzioni 
% data.Multibody.AlphaSwingArm       = angswing(xsol2(1));
% data.Multibody.GammaSteeringAxis   = xsol2(2)+gamma;
% data.Multibody.BetaFrame           = deg2rad(AngleFrameFinal+xsol2(2));
% %Passo e angolo mono non so se vanno salvati da qualche parte

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

