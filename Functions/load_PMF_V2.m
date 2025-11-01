
function data = load_PMF_V2()
    %This function make you choose a .pmf file and convert it in a struct
    %variable

    % Author: Antonio Filianoti
    
    %% IMPORT 
    
    % Path relativo alla cartella del progetto
    baseFolder = pwd;  % cartella corrente del progetto
    pmfFolder = fullfile(baseFolder, 'PMF Database');
    
    % Ask for a .pmf file in PMF Database
    [filename, position] = uigetfile(fullfile(pmfFolder, '*.pmf'), 'Select PMF file');
    
    
    % Full path del file scelto
    fullpath = fullfile(position, filename);
    
    % Estraggo solo il nome del file senza estensione
    [~, name_only, ~] = fileparts(filename);
    
    % Salvo il nome nella struct
    data.nome = name_only;
    
    % Carico il file .pmf
    load = textread(fullpath);


    %% SAVING 

    %% FRONT SUSPENSION
    data.FrontSuspension.ForksLength = load(1);
    data.FrontSuspension.ForksVerticalOffset = load(2);
    data.FrontSuspension.Compression = load(3);
    data.FrontSuspension.ForksMaxCompression = load(4);
    data.FrontSuspension.PlatesTopThickness = load(5);
    data.FrontSuspension.OffsetTop = load(6);
    data.FrontSuspension.OffsetTopMax = load(7);
    data.FrontSuspension.OffsetTopMin = load(8);
    data.FrontSuspension.PlatesBottomThickness = load(9);
    data.FrontSuspension.OffsetBottom = load(10);
    data.FrontSuspension.OffsetBottomMax = load(11);
    data.FrontSuspension.OffsetBottomMin = load(12);
    data.FrontSuspension.PlatesForkOffset = load(13);
    data.FrontSuspension.FootOffset = load(14);
    data.FrontSuspension.ForksWheelbase = load(15); % INTERASSE FORCHE 186;
    
    % controllo che l'interasse delle forche sia aggiornato se no lo
    % inizializzo
    if data.FrontSuspension.ForksWheelbase == 0
        data.FrontSuspension.ForksWheelbase = 180;
    end
    
    0.000000; % null
    data.FrontSuspension.COGX = load(17);
    data.FrontSuspension.COGZ = load(18);
    data.FrontSuspension.COGMass = load(19);
    0.000000; % null

    %% REAR SUSPENSION
    data.RearSuspension.LengthRearSuspension = load(21);
    data.RearSuspension.LengthMaxRearSuspension = load(22);
    data.RearSuspension.LengthMinRearSuspension = load(23);
    data.RearSuspension.Compression = load(24);
    data.RearSuspension.MaxCompressionRearSuspension = load(25);
    0.000000; % null
    data.RearSuspension.Type = load(33);

    % unitrack case
    if data.RearSuspension.Type == 4
        data.RearSuspension.TriangleSuspFrame = load(27);
        data.RearSuspension.TriangleFrameRod = load(28);
        data.RearSuspension.TriangleRodSusp = load(29);
    elseif data.RearSuspension.Type == 2 % polink case
        data.RearSuspension.TriangleSwingSusp = load(27);
        data.RearSuspension.TriangleSwingRod = load(28);
        data.RearSuspension.TriangleRodSusp = load(29);
    end

    data.RearSuspension.RodLength = load(30);
    data.RearSuspension.MaxRodLength = load(31);
    data.RearSuspension.MinRodLength = load(32);
   

    %% FRAME
    data.Frame.LengthRear2SteeringHead = load(36);
    data.Frame.HeightSteeringHead = load(37);
    data.Frame.DistSteeringHead = load(38); % distanza canotto sup e inf
    0.000000; % null
    0.000000; % null
    data.Frame.SteeringAxisTopOffset = load(41);
    data.Frame.SteeringAxisTopOffsetMax = load(42);
    data.Frame.SteeringAxisTopOffsetMin = load(43);
    0.000000; % null
    0.000000; % null
    data.Frame.SteeringAxisBottomOffset = load(46);
    data.Frame.SteeringAxisBottomOffsetMax = load(47);
    data.Frame.SteeringAxisBottomOffsetMin = load(48);
    0.000000; % null
    0.000000; % null
    data.Frame.AttFrameSupX = load(51);
    data.Frame.AttFrameSupZ = load(52);
    data.Frame.AngleAttFrameSupOffset = load(53);
    data.Frame.AttFrameSupOffset = load(54);
    data.Frame.AttFrameSupOffsetMax = load(55);
    data.Frame.AttFrameInfX = load(56);
    data.Frame.AttFrameInfZ = load(57);
    data.Frame.AngleAttFrameInfOffset = load(58);
    data.Frame.AttFrameInfOffset = load(59);
    data.Frame.AttFrameInfOffsetMax = load(60);
    data.Frame.PivotOffsetX = load(61);
    data.Frame.PivotOffsetZ = load(62);
    data.Frame.AnglePivotOffset = load(63);
    0.000000; % null
    0.000000; % null
    data.Frame.COGX = load(66);
    data.Frame.COGZ = load(67);
    data.Frame.COGMass = load(68);
    0.000000; % null
    0.000000; % null
    data.Frame.PinionX = load(71);
    data.Frame.PinionZ = load(72);
     data.Frame.PinionY = load(73); %  Y PIGNONE            
    0.000000; % null
    0.000000; % null
    data.Frame.FairingX1 = load(76);
    data.Frame.FairingZ1 = load(77);
    data.Frame.FairingX2 = load(78);
    data.Frame.FairingZ2 = load(79);
    0.000000;
    

    %% SWINGARM
    data.Swingarm.Length = load(81);
    data.Swingarm.AngleOffset = load(82);
    data.Swingarm.Offset = load(83);
    data.Swingarm.OffsetMax = load(84);
    0.000000; % null
    data.Swingarm.AttSwingInfX = load(86);
    data.Swingarm.AttSwingInfZ = load(87);
    data.Swingarm.AngleAttSwingInf = load(88);
    data.Swingarm.OffsetAttSwingInf = load(89);
    data.Swingarm.OffsetAttSwingInfMax = load(90);
    data.Swingarm.AttSwingSupX = load(91);
    data.Swingarm.AttSwingSupZ = load(92);
    data.Swingarm.AngleAttSwingSup = load(93);
    data.Swingarm.OffsetAttSwingSup = load(94);
    data.Swingarm.OffsetAttSwingSupMax = load(95);
    data.Swingarm.COGX = load(96);
    data.Swingarm.COGZ = load(97);
    data.Swingarm.COGMass = load(98);
    0.000000; % null
    0.000000; % null

    %% WHEELS
    data.Wheels.FrontSize = load(101);    
    data.Wheels.FrontMass = load(102);
    data.Wheels.RearSize = load(103);    
    data.Wheels.RearMass = load(104);
    
    data.Wheels.RearTorus = 50;
    data.Wheels.FrontTorus = 40;

    % creo la mesh delle gomme
    [data.Wheels.FineFrontMesh] = drawTorusMesh(1000, data.Wheels.FrontSize, data.Wheels.FrontTorus);
    [data.Wheels.CoarseFrontMesh] = drawTorusMesh(50, data.Wheels.FrontSize, data.Wheels.FrontTorus);
    [data.Wheels.FineRearMesh] = drawTorusMesh(1000, data.Wheels.RearSize, data.Wheels.RearTorus);
    [data.Wheels.CoarseRearMesh] = drawTorusMesh(50, data.Wheels.RearSize, data.Wheels.RearTorus);

  

    %% TRANSMISSION

    data.Transmission.FinalPinion = load(111);
    data.Transmission.FinalSprocket = load(112);
    data.Transmission.FinalChainModule = load(113);
    
    %% MULTIBODY
    data.Multibody.LengthSwingArmFinal = 0; % lunghezza effettiva forcellone
    data.Multibody.AlphaSwingArm = 0; % iclinazione effettiva forcellone
    data.Multibody.LengthFrameFinal = 0; % lunghezza effettiva telaio
    data.Multibody.BetaFrame = 0; % inclinazione effettiva telaio
    data.Multibody.GammaSteeringAxis = 0; % inclinazione canotto
    data.Multibody.mu = 0; % angolo pitch moto
    data.Multibody.Roll = 0; % angolo rollio moto
    data.Multibody.SteeringAngle = 0; % angolo sterzo
    data.Multibody.Yaw = 0; % angolo imbardata moto
    data.Multibody.YDis = 0; % spostamento laterale moto

    %% Road
    data.Road.Slope = 0;
    data.Road.Banking = 0;

    
    
end

