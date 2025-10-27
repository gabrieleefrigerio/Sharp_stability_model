function [Bike] = ModelInput(Bike)
%--------------------------------------------------------------------------
% MODELINPUT - Calcola i valori di velocità e il carico verticale anteriore
%              (Zf) per una gamma di velocità, tenendo conto degli effetti
%              aerodinamici.
%
% DESCRIZIONE:
%   La funzione genera un vettore di velocità comprese tra v_start e
%   v_finish, quindi calcola il carico verticale sull'asse anteriore (Zf)
%   includendo i contributi aerodinamici di portanza (Fl), resistenza (Fd)
%   e momento aerodinamico (Cm).
%
% INPUT:
%   v_start   [km/h]   - Velocità iniziale del range di analisi.
%   v_finish  [km/h]   - Velocità finale del range di analisi.
%   n_points  [ - ]    - Numero di punti (passi di velocità) da generare.
%   data      [struct] - Struttura contenente i parametri del veicolo:
%       • data.A_front      [m²]    Area frontale del veicolo
%       • data.Cd           [ - ]   Coefficiente di resistenza aerodinamica
%       • data.Cl           [ - ]   Coefficiente di portanza
%       • data.Cm           [ - ]   Coefficiente di momento aerodinamico
%       • data.corda        [m]     Corda aerodinamica
%       • data.Mf, data.Mr  [kg]    Masse sospese anteriore e posteriore
%       • data.g            [m/s²]  Accelerazione di gravità
%       • data.COG_total(1) [m]     Coordinata longitudinale del baricentro
%       • data.COG_total(3) [m]     Coordinata verticale del baricentro
%       • data.l1           [m]     Passo anteriore (distanza COG → asse ant.)
%
% OUTPUT:
%   vx_values [1 x n_points]  - Velocità in m/s (vettore riga)
%   Zf_values [1 x n_points]  - Carico verticale sull’asse anteriore [N]
%
% ESEMPIO:
%   [vx, Zf] = ModelInput(0, 200, 50, data);
%
%--------------------------------------------------------------------------

    % ---------------------- Parametri iniziali ----------------------------
    rho_air = 1.2;   % [kg/m³] densità dell'aria (condizioni standard)

    data = Bike.data;
    
    % ---------------------- Vettore velocità ------------------------------
    vx_kmh = linspace(Bike.Input.v_start, Bike.Input.v_finish, Bike.Input.n_points);  % velocità in km/h
    vx_values = vx_kmh(:)' / 3.6;                    % conversione → m/s (riga)

    % ---------------------- Forze aerodinamiche ---------------------------
    % Calcolo del termine costante: (1/2 * ρ * A_front)
    coeff_aero = 0.5 * rho_air * data.A_front;

    % Resistenza aerodinamica (Drag)
    Fd = coeff_aero * data.Cd .* vx_values.^2;       % [N]

    % Portanza aerodinamica (Lift)
    Fl = coeff_aero * data.Cl .* vx_values.^2;       % [N]

    % Momento aerodinamico rispetto al baricentro
    C_aero = coeff_aero * data.Cm .* vx_values.^2 .* data.corda;  % [N·m]

    % ---------------------- Carico verticale anteriore --------------------
    % Equilibrio dei momenti rispetto all’asse posteriore:
    %   Zf = [ ( (Mf+Mr)*g*x_COG - C_aero - Fl*x_COG - Fd*z_COG ) / l1 ]
    Zf = ((data.Mf + data.Mr) * data.g * data.COG_total(1) ...
         - C_aero ...
         - Fl * data.COG_total(1) ...
         - Fd * data.COG_total(3)) ...
         / data.l1;   % [N]

    % ---------------------- Uscite vettoriali -----------------------------
    Bike.Input.Zf_values = Zf(:)';   % assicurati che sia un vettore riga
    Bike.Input.vx_values = vx_values; 

end
