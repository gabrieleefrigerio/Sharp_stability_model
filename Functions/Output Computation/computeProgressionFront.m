function [trail,normtrail,caster_angle] = computeProgressionFront(data,Front_Susp_Compr,Second_Param,Second_Param_Type)
% COMPUTEPROGRESSIONFront Calcola la curva di come varia trail normtrail e caster.
%
% INPUT:
%   data - struct contenente le informazioni geometriche e cinematiche
%   Front_Susp_Compr - vettore riga di compressione forche valutate [mm]
%   Second_param - vettore riga con valori assunti dal secondo parametro.
%   Second_Param_Type - secondo parametro: "Steering Offset HF",
%                                          "Steering Offset P"
%                                          "Steering Offset LF"
%                                          "X Pivot"
%                                          "Y Pivot"
%                                          "Forks Vertical Offset"
%                                          "Rear Compression"
%                                          "Swingarm Fork pos"
%                                          "Rod Link"
%                                          "t1"   
%                                          "t2"
%                                          "t3"
%                                          "Roll"
%                                          "Steering Angle"
                                      %    "None"
%
% OUTPUT:
%  
%   trail,normtrail e caster angle - matrici
%                           su ogni riga valutazione  di tutte le
%                           compressioni compressioni anteriori
%                           per ogni valore del secondo parametro

% Author: Nicola Cabrini

% c'è il secondo parametro
if ~isequal(Second_Param_Type,"None")
    %inizializzo matrice escursione
    trail = zeros(length(Second_Param),length(Front_Susp_Compr));
    normtrail = zeros(length(Second_Param),length(Front_Susp_Compr));
    caster_angle = zeros(length(Second_Param),length(Front_Susp_Compr));
    
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
            case "Rear Compression"
                data.RearSuspension.Compression = Second_Param(ii);
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
        data.FrontSuspension.Compression = Front_Susp_Compr(1);
        
        [data, P] = cinematicClosures(data);

        [T, P, data] = MultibodyMatrices(data, P);

        [Output, P] = OutputMultibody(T, P, data,1);

        trail(ii,1) = Output.Trail;
        normtrail(ii,1) = Output.NormalTrail;
        caster_angle(ii,1) = Output.caster_angle;
    
        for jj = 2:length(Front_Susp_Compr)
            % assegno compressione che varia
            data.FrontSuspension.Compression = Front_Susp_Compr(jj);
            
            [data, P] = cinematicClosures(data);

            [T, P, data] = MultibodyMatrices(data, P);
    
            [Output, P] = OutputMultibody(T, P, data,1);
    
            trail(ii,jj) = Output.Trail;
            normtrail(ii,jj) = Output.NormalTrail;
            caster_angle(ii,jj) = Output.caster_angle;
        end
    end

% solo compressione mono come parametro
else
    %inizializzo vettore escursione
    trail = zeros(1,length(Front_Susp_Compr));
    normtrail = zeros(1,length(Front_Susp_Compr));
    caster_angle = zeros(1,length(Front_Susp_Compr));

    %calcolo nel primo punto per settare il riferimento
    data.FrontSuspension.Compression = Front_Susp_Compr(1);
    
    [data, P] = cinematicClosures(data);

        [T, P, data] = MultibodyMatrices(data, P);

        [Output, P] = OutputMultibody(T, P, data,1);

        trail(1) = Output.Trail;
        normtrail(1) = Output.NormalTrail;
        caster_angle(1) = Output.caster_angle;
    
    for jj = 2:length(Front_Susp_Compr)
        % assegno compressione che varia
        data.FrontSuspension.Compression = Front_Susp_Compr(jj);
    
        [data, P] = cinematicClosures(data);

        [T, P, data] = MultibodyMatrices(data, P);

        [Output, P] = OutputMultibody(T, P, data,1);

        trail(jj) = Output.Trail;
        normtrail(jj) = Output.NormalTrail;
        caster_angle(jj) = Output.caster_angle;
    end


end


end
