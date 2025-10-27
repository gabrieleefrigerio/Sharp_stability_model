function [outsquat,wheelbase,cogx,cogy,cogz] = computeProgressionRear(data,Rear_Susp_Compr,Second_Param,Second_Param_Type)
% COMPUTEPROGRESSIONSQUAT Calcola la curva di come varia lo squat ratio e altri output della moto.
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
%   outsquat ecc - matrici
%                           su ogni riga valutazione  di tutte le compressioni compressioni posteriori
%                           per ogni valore del secondo parametro

% Author: Nicola Cabrini

% c'è il secondo parametro
if ~isequal(Second_Param_Type,"None")
    %inizializzo matrice escursione
    outsquat = zeros(length(Second_Param),length(Rear_Susp_Compr));
    wheelbase = zeros(length(Second_Param),length(Rear_Susp_Compr));
    cogx = zeros(length(Second_Param),length(Rear_Susp_Compr));
    cogy = zeros(length(Second_Param),length(Rear_Susp_Compr));
    cogz = zeros(length(Second_Param),length(Rear_Susp_Compr));

    
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
        
        [data, P] = cinematicClosures(data);

        [T, P, data] = MultibodyMatrices(data, P);

        [Output, P] = OutputMultibody(T, P, data,1);

        outsquat(ii,1) = Output.SquatRatio;
        wheelbase(ii,1) = Output.Wheelbase;
        cogx(ii,1) = Output.COG(1);
        cogy(ii,1) = Output.COG(2);
        cogz(ii,1) = Output.COG(3);
    
        for jj = 2:length(Rear_Susp_Compr)
            % assegno compressione che varia
            
            data.RearSuspension.Compression = Rear_Susp_Compr(jj);
            [data, P] = cinematicClosures(data);
    
            [T, P, data] = MultibodyMatrices(data, P);
    
            [Output, P] = OutputMultibody(T, P, data,1);
    
            outsquat(ii,jj) = Output.SquatRatio;
            wheelbase(ii,jj) = Output.Wheelbase;
            cogx(ii,jj) = Output.COG(1);
            cogy(ii,jj) = Output.COG(2);
            cogz(ii,jj) = Output.COG(3);
        end
    end

% solo compressione mono come parametro
else
    %inizializzo vettore escursione
    outsquat = zeros(1,length(Rear_Susp_Compr));
    wheelbase = zeros(1,length(Rear_Susp_Compr));
    cogx = zeros(1,length(Rear_Susp_Compr));
    cogy = zeros(1,length(Rear_Susp_Compr));
    cogz = zeros(1,length(Rear_Susp_Compr));
    
    %calcolo nel primo punto per settare il riferimento
    data.RearSuspension.Compression = Rear_Susp_Compr(1);
    
    [data, P] = cinematicClosures(data);

    [T, P, data] = MultibodyMatrices(data, P);

    [Output, P] = OutputMultibody(T, P, data,1);


    outsquat(1) = Output.SquatRatio;
    wheelbase(1) = Output.Wheelbase;
    cogx(1) = Output.COG(1);
    cogy(1) = Output.COG(2);
    cogz(1) = Output.COG(3);
    
    for jj = 2:length(Rear_Susp_Compr)
        % assegno compressione che varia

        data.RearSuspension.Compression = Rear_Susp_Compr(jj);
    
        [data, P] = cinematicClosures(data);

        [T, P, data] = MultibodyMatrices(data, P);

        [Output, P] = OutputMultibody(T, P, data,1);

        outsquat(jj) = Output.SquatRatio;
        wheelbase(jj) = Output.Wheelbase;
        cogx(jj) = Output.COG(1);
        cogy(jj) = Output.COG(2);
        cogz(jj) = Output.COG(3);
    end


end


end
