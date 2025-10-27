function [Vertical_Wheel_Travel, Normalized, Swingarm_Angle] = computeProgression(data,Rear_Susp_Compr,Second_Param,Second_Param_Type)
% COMPUTEPROGRESSION Calcola la curva di progressione della moto.
%
% INPUT:
%   data - struct contenente le informazioni geometriche e cinematiche
%   Rear_Susp_Compr - vettore riga di compressione mono valutate [mm]
%   Second_param - vettore riga con valori assunti dal secondo parametro.
%   Second_Param_Type - secondo parametro: "Steering Offset HF",
%                                          "Steering Offset P"
%                                          "Steering Offset LF"
%                                          "X Pivot"
%                                          "Y Pivot"
%                                          "Forks Vertical Offset"
%                                          "Front Compression"
%                                          "Swingarm Fork pos"
%                                          "Rod Link"
%                                          "t1"   %da mettere, capire come gestirli con i due schemi
%                                          "t2"
%                                          "t3"
%                                          "Roll"
%                                          "Steering Angle"
                                      %    "None"
%
% OUTPUT:
%  
%   Vertical_Wheel_Travel - matrice di escursione verticale ruota [mm]
%                           su ogni riga valutazione  di tutte le compressioni compressioni posteriori
%                           per ogni valore del secondo parametro
%   Normalized            - matrice che è la derivata della curva
%   Swingarm_Angle        - angolo forcellone [deg] su ogni riga valutazione  di tutte le compressioni compressioni posteriori
%                           per ogni valore del secondo parametro 

% Author: Nicola Cabrini

% c'è il secondo parametro
if ~isequal(Second_Param_Type,"None")
    %inizializzo matrice escursione
    Vertical_Wheel_Travel = zeros(length(Second_Param),length(Rear_Susp_Compr));
    Swingarm_Angle = zeros(length(Second_Param),length(Rear_Susp_Compr));

    %inizializzo vettore riferimenti (escursione a 0 a prima compressione
    %mono valutata)
    ref0 = zeros(1,length(Second_Param));
    
    for ii = 1:length(Second_Param)
        switch Second_Param_Type
            case "Steering Offset HF"
                data.Frame.SteeringAxisTopOffset = Second_Param(ii);
            case "Steering Offset LF"
                data.Frame.SteeringAxisBottomOffset = Second_Param(ii);
            case "Steering Offset P"
                data.FrontSuspension.OffsetTop = Second_Param(ii);
                data.FrontSuspension.OffsetBottom = Second_Param(ii);
            case  "X Pivot"
                data.Frame.PivotOffsetX = Second_Param(ii);
            case "Y Pivot"
                 data.Frame.PivotOffsetZ = Second_Param(ii);
            case "Forks Vertical Offset"
                data.FrontSuspension.ForksVerticalOffset = Second_Param(ii);
            case "Front Compression"
                data.FrontSuspension.Compression = Second_Param(ii);
            case "Swingarm Fork pos"
                data.Swingarm.Offset = Second_Param(ii);
            case "Rod Link"
                data.RearSuspension.RodLength = Second_Param(ii);
            case "t1-(susp-frame/swing-susp)"
                if data.RearSuspension.Type == 4
                    data.RearSuspension.TriangleSuspFrame = Second_Param(ii);
                elseif data.RearSuspension.Type == 2 % polink case
                    data.RearSuspension.TriangleSwingSusp = Second_Param(ii);
                end
            case "t2-(frame-rod/swing-rod)"
                if data.RearSuspension.Type == 4
                    data.RearSuspension.TriangleFrameRod = Second_Param(ii);
                elseif data.RearSuspension.Type == 2 % polink case
                    data.RearSuspension.TriangleSwingRod = Second_Param(ii);
                end
            case "t3-(rod-susp)"
                if data.RearSuspension.Type == 4
                    data.RearSuspension.TriangleRodSusp = Second_Param(ii);
                elseif data.RearSuspension.Type == 2 % polink case
                    data.RearSuspension.TriangleRodSusp = Second_Param(ii);
                end
            case "Roll"
                data.Multibody.Roll = Second_Param(ii);
            case "Steering Angle"
                data.Multibody.Roll = Second_Param(ii);
        end

        %calcolo nel primo punto per settare il riferimento
        data.RearSuspension.Compression = Rear_Susp_Compr(1);
        
        % chiusure cinamatiche per calcolo angolo forcellone 
        [data,~] = cinematicClosures(data);
        
        % riferimento a 0 per prima compressione valutata
        ref0(ii) = sin(data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal;
        Vertical_Wheel_Travel(ii,1) = 0;
        Swingarm_Angle(ii,1) = data.Multibody.AlphaSwingArm;
    
        for jj = 2:length(Rear_Susp_Compr)
            % assegno compressione che varia
            data.RearSuspension.Compression = Rear_Susp_Compr(jj);
        
            % chiusure cinamatiche per calcolo angolo forcellone 
            [data,~] = cinematicClosures(data);
            
            Vertical_Wheel_Travel(ii,jj) = sin(data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal - ref0(ii);

            Swingarm_Angle(ii,jj) = data.Multibody.AlphaSwingArm;
        end
    end

% solo compressione mono come parametro
else
    %inizializzo vettore escursione
    Vertical_Wheel_Travel = zeros(1,length(Rear_Susp_Compr));
    Swingarm_Angle = zeros(1,length(Rear_Susp_Compr));
    
    %calcolo nel primo punto per settare il riferimento
    data.RearSuspension.Compression = Rear_Susp_Compr(1);
    
    % chiusure cinamatiche per calcolo angolo forcellone 
    [data,~] = cinematicClosures(data);
    
    % riferimento a 0 per prima compressione valutata
    ref0 = sin(data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal;
    Vertical_Wheel_Travel(1) = 0;
    Swingarm_Angle(1) = data.Multibody.AlphaSwingArm;

    for jj = 2:length(Rear_Susp_Compr)
        % assegno compressione che varia
        data.RearSuspension.Compression = Rear_Susp_Compr(jj);
    
        % chiusure cinamatiche per calcolo angolo forcellone 
        [data,~] = cinematicClosures(data);
        
        Vertical_Wheel_Travel(jj) = sin(data.Multibody.AlphaSwingArm) * data.Multibody.LengthSwingArmFinal - ref0;
        Swingarm_Angle(jj) = data.Multibody.AlphaSwingArm;
    end


end

Swingarm_Angle = abs(rad2deg(Swingarm_Angle));
Normalized = diff(Vertical_Wheel_Travel,1,2)./diff(Rear_Susp_Compr); %lungo uno in meno

%riuniformo dimensioni
Normalized = [Normalized,Normalized(:,end)];



end

