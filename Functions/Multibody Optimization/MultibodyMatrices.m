function [T_out, P, data] = MultibodyMatrices(data,P) %data,P
% MULTIBODYMATRICES - Computes the transformation matrices and key points of a motorcycle's front and rear suspension system.
%
% This function calculates the spatial configuration of the motorcycle multibody system 
% given its geometric and kinematic parameters. It returns both the homogeneous transformation 
% matrices and the 3D coordinates of important reference points.
%
% INPUT:
%   data : struct
%       Contains all the necessary geometric and kinematic parameters of the motorcycle, 
%       including suspension lengths, angles, wheel sizes, and offsets.
%
% OUTPUT:
%   T_out : struct
%       Contains 4x4 homogeneous transformation matrices of the various components of the motorcycle.
%       These include:
%           - Ground reference (SDR)
%           - Wheel centers
%           - Suspension pivots and frames
%           - Fork plates and fork legs
%
%   P_out : struct
%       Contains 3D coordinates (as [3x1] vectors) of key points of the motorcycle model.
%       These include:
%           - Wheel contact points
%           - Pivot points
%           - Fork and suspension related points
%
% AUTHOR:
%   Antonio Filianoti

% -------------------------------------------------------------------------
    %% === ESTRAZIONE PARAMETRI ===

    RoadSlope           = data.Road.Slope;
    Banking             = data.Road.Banking;
    Roll                = data.Multibody.Roll;
    AlphaSwingArm       = data.Multibody.AlphaSwingArm;
    gamma               = data.Multibody.Gamma;  %% gamma regolazione dovuta a piastre
    SteeringAngle       = data.Multibody.SteeringAngle;
    LengthSwingArmFinal = data.Multibody.LengthSwingArmFinal;
    BetaSDRFrame       = data.Multibody.BetaSDRFrame;         
    LengthFrameFinal    = data.Multibody.LengthFrameFinal;
    RearSize            = data.Wheels.RearSize;
    RearTorus           = data.Wheels.RearTorus;
    FrontSize           = data.Wheels.FrontSize;
    FrontTorus          = data.Wheels.FrontTorus;
    PlatesForkOffset    = data.FrontSuspension.PlatesForkOffset;
    ForksWheelbase      = data.FrontSuspension.ForksWheelbase;
    OffsetTop           = data.FrontSuspension.OffsetTop;
    PlatesTopThickness  = data.FrontSuspension.PlatesTopThickness;
    ForksVerticalOffset = data.FrontSuspension.ForksVerticalOffset;
    ForksLength         = data.FrontSuspension.ForksLength;
    ForksCompression    = data.FrontSuspension.Compression;
    FootOffset          = data.FrontSuspension.FootOffset;
    MassRear            = data.Swingarm.COGMass;
    MassFront           = data.FrontSuspension.COGMass;
    MassFrame           = data.Frame.COGMass;
    MassRearWheel       = data.Wheels.RearMass;
    MassFrontWheel      = data.Wheels.FrontMass;
    COGXRear            = data.Swingarm.COGX;
    COGZRear            = data.Swingarm.COGZ;
    COGXFrame           = data.Frame.COGX;
    COGZFrame           = data.Frame.COGZ;
    COGXFront           = data.FrontSuspension.COGX;
    COGZFront           = data.FrontSuspension.COGZ;


    %% === OPERATORI ===
    [Rx, Ry, Rz, T_rot, T_tr] = getOperators();

    
    %% === Trasformazioni ===
    % Piano stradale inclinato
    T_SDR_abs = T_rot(Rx(Banking)) * T_rot(Ry(RoadSlope));
    T_SDR_abs(1:3,4) = [0; 0; 0];
    
    % Orientamento iniziale toroide (centrato e con rollio)
    T_roll = T_SDR_abs * T_rot(Rx(Roll));
    T_torus_init = T_roll * T_tr(RearTorus * [0; 0; 1]);
    
    %% === Generazione punti toroide (locali) ===
    P_local_rear = data.Wheels.FineRearMesh;


    %% === Allineamento toroide tangente al piano ===
    %bypasso se moto dritta per velocità
    if data.Multibody.Roll == 0 && data.Multibody.SteeringAngle == 0 && data.Road.Slope == 0 && data.Road.Banking == 0
        % INVERSA del piano e normale globale
        T_inv_plane  = inv(T_SDR_abs);
        plane_normal = T_SDR_abs(1:3,3);
        
        % OPERATORE di traslazione omogenea
        T_tr = @(v) [eye(3), v; zeros(1,3), 1];
        
        % TROVO il punto più basso del toroide nel sistema locale
        [~, idx_min] = min(P_local_rear(3,:));
        p_local_lowest = P_local_rear(:, idx_min);
        
        % CALCOLO la traslazione necessaria per portare il punto più basso sul piano
        % Trasformo il punto più basso nel sistema del piano
        p_plane_lowest = T_inv_plane * (T_torus_init * p_local_lowest);
        
        % L'offset di traslazione è l'opposto della coordinata Z
        dz_opt = -p_plane_lowest(3);
        
        % COSTRUISCO la trasformazione finale
        T_centro_post = T_torus_init * T_tr(dz_opt * plane_normal);
        
        % CALCOLO il punto di contatto finale (lo stesso punto, traslato)
        p_contact = T_centro_post * p_local_lowest;
        p_contatto_post = p_contact(1:3); % Estrae le coordinate 3D
    else
        [T_centro_post, p_contatto_post] = alignTorusTangentialToPlane(T_torus_init, T_SDR_abs, P_local_rear);
    end

    p_centro_post = T_centro_post(1:3,4);
    p_sprocket = [p_centro_post(1); data.Frame.PinionY; p_centro_post(3)];
    T_contatto_post = T_centro_post;
    T_contatto_post(1:3,4) = p_contatto_post;
    
    % --- Calcolo diametro e raggio primitivo della corona ---
    z_sprocket = data.Transmission.FinalSprocket;   % numero denti corona
    m_chain    = data.Transmission.FinalChainModule; % passo catena
    d_sprocket = m_chain * z_sprocket;
    r_sprocket = d_sprocket / 2/ pi;
    
    % --- Costruzione trasformazione della corona (origine centro ruota posteriore) ---
    % Traslazione laterale lungo Y del mozzo posteriore
    T_sprocket = T_centro_post * T_tr(data.Frame.PinionY * [0;1 ; 0]);

    T_sup_sprocket = T_sprocket * T_tr(r_sprocket * [0;0 ; 1]);
    
    % --- Punto superiore della corona ---
    p_sup_sprocket = T_sup_sprocket(1:3,4);
    
    
    %% Ottimizzazione angolo beccheggio
    P_local_front = data.Wheels.FineFrontMesh;
    
    % if Roll == 0 && SteeringAngle== 0 
    %     mu_opt = 0;
    % else
    %     mu_opt = findMuForFlatFrontContact(T_centro_post, T_SDR_abs, P_local_front, data,P);
    % end

    mu_opt = 0;
    data.Multibody.mu = rad2deg(mu_opt);
    T_centro_post = T_centro_post * T_rot(Ry(mu_opt));

    %% === FORCELLONE POST ===
    T_mid = T_centro_post * T_rot(Ry(AlphaSwingArm));
    T_swing_pivot = T_mid * T_tr([LengthSwingArmFinal; 0; 0]);
    p_pivot = T_swing_pivot(1:3,4);

   
    T_COG_rear = T_swing_pivot * T_tr([COGXRear; 0 ; COGZRear]);
    p_COG_rear = T_COG_rear(1:3,4);

    %% === TELAIO PIGNONE E ASSE DI STERZO ===
    T_frame_pivot = T_swing_pivot * T_rot(Ry(-AlphaSwingArm));
    T_frame_pivot = T_frame_pivot * T_rot(Ry(-BetaSDRFrame));


    % Trasformazione 4x4 del pignone (orientamento pari al pivot)
    v_pinion_local = [data.Frame.PinionX; data.Frame.PinionY; data.Frame.PinionZ];
    T_pinion = T_frame_pivot * T_tr(v_pinion_local);
    p_pinion = T_pinion(1:3,4);

    
    % --- Calcolo raggio primitivo e diametro ---
    z_pinion = data.Transmission.FinalPinion;
    d_pinion = m_chain * z_pinion / pi;
    r_pinion = d_pinion / 2;
    
    % --- Punto superiore del pignone ---
    T_sup_pinion = T_pinion * T_tr(r_pinion * [0;0;1]);
    p_sup_pinion = T_sup_pinion(1:3,4);



    T_COG_frame = T_frame_pivot * T_tr([COGXFrame; 0 ; COGZFrame]); %sono x e z risp sdr sbagliato
    p_COG_frame = T_COG_frame(1:3,4);

    T_steering_axis = T_frame_pivot * T_tr([P.Coordinate2D.l_frame; 0; P.Coordinate2D.h_frame]);
    T_steering_axis = T_steering_axis * T_rot(Ry(-gamma)) * T_rot(Rz(SteeringAngle));
    p_steering_axis = T_steering_axis(1:3,4);


    %% === PIATTI FORCELLA DX / SX ===
    v_dx = [PlatesForkOffset + OffsetTop; -ForksWheelbase/2; PlatesTopThickness]; %%
    v_sx = [PlatesForkOffset + OffsetTop;  ForksWheelbase/2; PlatesTopThickness];
    
    T_piastra_sup_dx = T_steering_axis * T_tr(v_dx);
    p_piastra_sup_dx = T_piastra_sup_dx(1:3,4);
    
    T_piastra_sup_sx = T_steering_axis * T_tr(v_sx);
    p_piastra_sup_sx = T_piastra_sup_sx(1:3,4);
    
    %% === CORPO FORCELLA === 
    T_sup_forc_dx = T_piastra_sup_dx * T_tr([0;0;ForksVerticalOffset]);
    p_sup_forc_dx = T_sup_forc_dx(1:3,4);
    
    T_sup_forc_sx = T_piastra_sup_sx * T_tr([0;0;ForksVerticalOffset]);
    p_sup_forc_sx = T_sup_forc_sx(1:3,4);
    
    length_1 = (7/10)*(ForksLength);
    
    T_mid_forc_dx = T_sup_forc_dx * T_tr([0; 0; -length_1]);
    p_mid_forc_dx = T_mid_forc_dx(1:3,4);
    
    T_mid_forc_sx = T_sup_forc_sx * T_tr([0; 0; -length_1]);
    p_mid_forc_sx = T_mid_forc_sx(1:3,4);
    
    length_2 = (3/10)*(ForksLength) - ForksCompression;
    
    T_end_forc_dx = T_mid_forc_dx * T_tr([0; 0; -length_2]);
    p_end_forc_dx = T_end_forc_dx(1:3,4);
    
    T_end_forc_sx = T_mid_forc_sx * T_tr([0; 0; -length_2]);
    p_end_forc_sx = T_end_forc_sx(1:3,4);
    
    %% === PIEDINI FORCELLA ===
    T_foot_dx = T_end_forc_dx * T_tr([FootOffset; 0; 0]);
    p_foot_dx = T_foot_dx(1:3,4);
    
    T_foot_sx = T_end_forc_sx * T_tr([FootOffset; 0; 0]);
    p_foot_sx = T_foot_sx(1:3,4);
    
    %% === CENTRO RUOTA ANTERIORE ===
    delta = dot(p_foot_sx - p_foot_dx, T_foot_dx(1:3,2));
    p_centro_ant = p_foot_dx + 0.5 * delta * T_foot_dx(1:3,2);
    
    T_centro_ant = T_foot_dx;
    T_centro_ant(1:3,4) = p_centro_ant;
    
    %% === COG FRONT ===
    T_COG_front = T_centro_ant * T_tr([COGXFront; 0 ; COGZFront]);
    p_COG_front = T_COG_front(1:3,4);

    % %% === PIATTI FORCELLA DX / SX ===
    % R_steering_axis = T_steering_axis(1:3,1:3);
    % 
    % v_dx = [PlatesForkOffset + OffsetTop; -ForksWheelbase/2; 0];
    % v_sx = [PlatesForkOffset + OffsetTop;  ForksWheelbase/2; 0];
    % 
    % p_piastra_sup_dx = p_steering_axis + R_steering_axis * v_dx;
    % p_piastra_sup_sx = p_steering_axis + R_steering_axis * v_sx;
    % 
    % T_piastra_sup_dx = [R_steering_axis, p_piastra_sup_dx; 0 0 0 1];
    % T_piastra_sup_sx = [R_steering_axis, p_piastra_sup_sx; 0 0 0 1];
    % 
    % %% === PIATTI FORCELLA DX / SX ===
    % v_dx(3) =  ForksVerticalOffset;
    % v_sx(3) = ForksVerticalOffset;
    % 
    % p_sup_forc_dx = p_steering_axis + R_steering_axis * v_dx;
    % p_sup_forc_sx = p_steering_axis + R_steering_axis * v_sx;
    % 
    % T_sup_forc_dx = [R_steering_axis, p_sup_forc_dx; 0 0 0 1];
    % T_sup_forc_sx = [R_steering_axis, p_sup_forc_sx; 0 0 0 1];
    % 
    % %% === CORPO FORCELLA ===
    % v_trasl_mid = - (7/10) * ForksLength - ForksCompression;
    % p_mid_forc_dx = p_sup_forc_dx + R_steering_axis(:,3) * v_trasl_mid;
    % p_mid_forc_sx = p_sup_forc_sx + R_steering_axis(:,3) * v_trasl_mid;
    % 
    % T_mid_forc_dx = [R_steering_axis, p_mid_forc_dx; 0 0 0 1];
    % T_mid_forc_sx = [R_steering_axis, p_mid_forc_sx; 0 0 0 1];
    % 
    % v_trasl_end = -ForksLength;
    % p_end_forc_dx = p_sup_forc_dx + R_steering_axis(:,3) * v_trasl_end;
    % p_end_forc_sx = p_sup_forc_sx + R_steering_axis(:,3) * v_trasl_end;
    % 
    % T_end_forc_dx = [R_steering_axis, p_end_forc_dx; 0 0 0 1];
    % T_end_forc_sx = [R_steering_axis, p_end_forc_sx; 0 0 0 1];
    % 
    % %% === PIEDINI FORCELLA ===
    % p_foot_dx = p_end_forc_dx + R_steering_axis(:,1) * FootOffset;
    % p_foot_sx = p_end_forc_sx + R_steering_axis(:,1) * FootOffset;
    % 
    % T_foot_dx = [R_steering_axis, p_foot_dx; 0 0 0 1];
    % T_foot_sx = [R_steering_axis, p_foot_sx; 0 0 0 1];
    % 
    % %% === CENTRO RUOTA ANTERIORE ===
    % delta = dot(p_foot_sx - p_foot_dx, T_foot_dx(1:3,2));
    % p_centro_ant = p_foot_dx + 0.5 * delta * T_foot_dx(1:3,2);
    % T_centro_ant = T_foot_dx;
    % T_centro_ant(1:3,4) = p_centro_ant;
    % 
    % T_COG_front = T_centro_ant * T_tr([COGXFront; 0 ; COGZFront]);
    % p_COG_front = T_COG_front(1:3,4);
    % 

    %% === PTO CONTATTO ANT ===
    P_Front_Torus = drawTorus([], T_centro_ant,data.Wheels.FineFrontMesh, false);
    
    % Definizione del piano nel sistema di riferimento T_SDR_abs
    p0 = T_SDR_abs(1:3, 4);       % Punto sul piano
    n = T_SDR_abs(1:3, 3);        % Normale al piano
    
    % Calcolo delle distanze dei punti dal piano
    diff = P_Front_Torus(1:3, :) - p0;                     % Vettori dal punto del piano ai punti P
    distances = abs(n' * diff);               % Distanze scalari dei punti dal piano
    
    % Trovare il punto di contatto anteriore
    [~, ind_cont_ant] = min(distances);
    p_contatto_ant = P_Front_Torus(1:3, ind_cont_ant);
    
    % Aggiornare la trasformazione T_contatto_ant
    T_contatto_ant = T_centro_ant;
    T_contatto_ant(1:3, 4) = p_contatto_ant;


    %% === PTO COG TOT ===
    p_COG_tot = (MassRear * p_COG_rear + MassFront * p_COG_front + MassFrame * p_COG_frame + MassFrontWheel * p_centro_ant + MassRearWheel * p_centro_post) / ...
            (MassRear + MassFront + MassFrame + MassFrontWheel + MassRearWheel);
    COG_front_assembl = (MassFront * p_COG_front + MassFrontWheel * p_centro_ant) / ...
            (MassFront + MassFrontWheel);
    COG_rear_assembl = (MassRear * p_COG_rear + MassFrame * p_COG_frame + MassRearWheel * p_centro_post) / ...
            (MassRear + MassFrame + MassRearWheel);

    %% PUNTI TIRO CATENA
    % --- Calcolo punti di tangenza catena ---
    % [p_chain_inf_sprocket, p_chain_inf_pinion] = chainTangentPoints(p_sprocket, r_sprocket, p_pinion, r_pinion, -1);


    % Assegna ogni campo manualmente
    P.p_contatto_post   = p_contatto_post;
    P.p_centro_post     = p_centro_post;
    P.p_pivot           = p_pivot;
    P.p_pinion          = p_pinion;
    P.p_sprocket        = p_sprocket;
    P.p_sup_pinion      = p_sup_pinion;
    P.p_sup_sprocket    = p_sup_sprocket;
    P.p_steering_axis   = p_steering_axis;
    
    P.p_piastra_sup_dx  = p_piastra_sup_dx;
    P.p_piastra_sup_sx  = p_piastra_sup_sx;
    
    P.p_sup_forc_dx     = p_sup_forc_dx;
    P.p_sup_forc_sx     = p_sup_forc_sx;
    
    P.p_mid_forc_dx     = p_mid_forc_dx;
    P.p_mid_forc_sx     = p_mid_forc_sx;
    
    P.p_end_forc_dx     = p_end_forc_dx;
    P.p_end_forc_sx     = p_end_forc_sx;
    
    P.p_foot_dx         = p_foot_dx;
    P.p_foot_sx         = p_foot_sx;
    
    P.p_centro_ant      = p_centro_ant;
    P.p_contatto_ant    = p_contatto_ant;
    
    P.p_COG_tot         = p_COG_tot;
    P.p_COG_rear        = p_COG_rear;
    P.p_COG_front       = p_COG_front;
    P.p_COG_frame       = p_COG_frame;
    P.COG_rear_assembl  = COG_rear_assembl;
    P.COG_front_assembl = COG_front_assembl;
    %% === Proiezione di tutti i punti nel piano della moto ===
    
    % spostamento lungo z locale 
    z_axis = T_centro_post(1:3,3); 
    % asse z del frame moto 
    T_moto = T_centro_post;

    P.bikePlane.p_contatto_post = projectToBikePlane(P.p_contatto_post, T_moto);
    % 
    dz = abs(P.bikePlane.p_contatto_post(3));
    T_moto = T_moto * T_tr(-dz * [0;0;1]); % traslazione

    fields = fieldnames(P);
    
    for i = 1:numel(fields)
        fname = fields{i};
        if startsWith(fname, 'p_')   % considero solo i punti (ignoro eventuali altri campi)
            p = P.(fname);
            P.bikePlane.(fname) = projectToBikePlane(p, T_moto);
        end
    end

    %% === STRUCT DI OUTPUT ===
    T_out = struct(...
        'T_SDR_abs', T_SDR_abs, ...
        'T_bikePlane',T_moto,...
        'T_roll', T_roll, ...
        'T_contatto_post', T_contatto_post, ...
        'T_centro_post', T_centro_post, ...
        'T_swing_pivot', T_swing_pivot, ...
        'T_frame_pivot', T_frame_pivot, ...
        'T_sprocket', T_sprocket, ...
        'T_pinion', T_pinion,...
        'T_steering_axis', T_steering_axis, ...
        'T_piastra_sup_dx',T_piastra_sup_dx,...
        'T_piastra_sup_sx',T_piastra_sup_sx,...
        'T_sup_forc_dx', T_sup_forc_dx, ...
        'T_sup_forc_sx', T_sup_forc_sx, ...
        'T_mid_forc_dx', T_mid_forc_dx, ...
        'T_mid_forc_sx', T_mid_forc_sx, ...
        'T_end_forc_dx', T_end_forc_dx, ...
        'T_end_forc_sx', T_end_forc_sx, ...
        'T_foot_dx', T_foot_dx, ...
        'T_foot_sx', T_foot_sx, ...
        'T_centro_ant', T_centro_ant, ...
        'T_contatto_ant', T_contatto_ant, ...
        'T_COG_rear', T_COG_rear,...
        'T_COG_frame', T_COG_frame,...
        'T_COG_front', T_COG_front...
    );



end



