function [A] = Sharp_Model(Bike, ii)
    % Input
    % vx = forward speed
    % Zf = front tire load

    %% PARAMETRI
    % Mass
    Mf = 30.65; % mass front part
    Mr = 217.45;
    Ifx = 1.23; % kg·m², da 23 slug·ft²
    Ifz = 0.44; % kg·m²
    Irx = 31.20; % kg·m²
    Irz = 21.08; % assicurati del valore corretto
    Ify = 0.72; % kg·m²
    Iry = 1; % kg·m²
    Crxz = 1.74; % kg·m²
    
    data = Bike.data;
    vx = Bike.Input.vx_values(ii);
    Zf = Bike.Input.Zf_values(ii);

    % Geometrical Parameters in m
    k = data.k / 1000;
    e = data.e/ 1000;
    j = data.j/ 1000;
    h = data.h/ 1000;
    epsilon = deg2rad(data.epsilon);
    l = data.l/ 1000;
    b = data.b/ 1000;
    Rr = data.Rr/ 1000;
    Rf = data.Rf/ 1000;
    t = data.t/ 1000;

    % Geometrical Parameters in m
    % k = 0.6572;
    % e = 0.024;
    % j = 0.85226;
    % h = 0.66;
    % epsilon = deg2rad(27);
    % l = 1.414-0.48;
    % b = 0.48;
    % Rr = 0.305;
    % Rf = 0.305;
    % t = 0.116*cos(epsilon);

    % Tires
    sigmaf = 0.8 * 0.3048; %m
    sigmar = 0.8 * 0.3048; %m
    Cf1 = 2512*4.44822; % N·m/rad
    Cf2 = 211*4.44822;
    Cr1 = 3559*4.44822;
    Cr2 = 298*4.44822;

    % gravity acceleration
    g = 9.81;
    % steering damper
    K = 6.779;
    
    %% MATRICE M
    
    % riga 1
    m11 = Mf + Mr;
    m12 = Mf * k;
    m13 = 0;
    m14 = 0;
    m15 = Mf * e;
    m16 = Mf * j + Mr * h;
    
    % riga 2
    m21 = Mf * k;
    m22 = Mf * k^2 + Irz + Ifx * sin(epsilon)^2 + Ifz * cos(epsilon)^2;
    m23 = 0;
    m24 = 0;
    m25 = Mf * e * k + Ifz * cos(epsilon);
    m26 = Mf * j * k - Crxz + (Ifz - Ifx) * sin(epsilon) * cos(epsilon);

    % riga 3
    m31 = 0;
    m32 = 0;
    m33 = sigmaf / vx;
    m34 = 0;
    m35 = 0;
    m36 = 0;

    % riga 4
    m41 = 0;
    m42 = 0;
    m43 = 0;
    m44 = sigmar / vx;
    m45 = 0;
    m46 = 0;

    % riga 5
    m51 = Mf * e;
    m52 = Mf * e * k + Ifz * cos(epsilon);
    m53 = 0;
    m54 = 0;
    m55 = Ifz + Mf * e^2;
    m56 = Mf * e * j + Ifz * sin(epsilon);
    
    % riga 6
    m61 = Mf * j + Mr * h;
    m62 = Mf * j * k - Crxz + (Ifz - Ifx) * sin(epsilon) * cos(epsilon);
    m63 = 0;
    m64 = 0;
    m65 = Mf * e * j + Ifz * sin(epsilon);
    m66 = Mf * j^2 + Mr * h^2 + Irx + Ifx * cos(epsilon)^2 + Ifz * sin(epsilon)^2;
    %m66 = Mf * j^2 + Mr * h^2 + Irx + Ifx * (cos(epsilon)^2 + Ifz * sin(epsilon)^2);
    %CONTROLLARE QUESTA RIGA
    
    % Costruzione matrice M
    M = [
        m11, m12, m13, m14, m15, m16;
        m21, m22, m23, m24, m25, m26;
        m31, m32, m33, m34, m35, m36;
        m41, m42, m43, m44, m45, m46;
        m51, m52, m53, m54, m55, m56;
        m61, m62, m63, m64, m65, m66
    ];
    
    %% MATRICE H
    
    % riga 1
    h11 = 0;
    h12 = (Mf + Mr) * vx;
    h13 = -1;
    h14 = -1;
    h15 = 0;
    h16 = 0;
    
    % riga 2
    h21 = 0;
    h22 = Mf * k * vx;
    h23 = -l;
    h24 = b;
    h25 = -Ify / Rr * sin(epsilon) * vx;
    h26 = -(Ify / Rf - Iry / Rr) * vx;
    h26 = -(Ify / Rf + Iry / Rr) * vx;
    
    % riga 3
    h31 = Cf1 / vx;
    h32 = Cf1 * l / vx;
    h33 = 1;
    h34 = 0;
    h35 = -Cf1 * t / vx;  
    h36 = 0;
    
    % riga 4
    h41 = Cr1 / vx;
    h42 = -Cr1 * b / vx;
    h43 = 0;
    h44 = 1;
    h45 = 0;
    h46 = 0;

    % riga 6
    h61 = 0;
    h62 = (Mf * j + Mr * h + Ify / Rf + Iry / Rr) * vx;
    h63 = 0;
    h64 = 0;
    h65 = Ify / Rf * cos(epsilon) * vx;
    h66 = 0;

    % riga 5
    h51 = 0;
    h52 = (Mf * e + Ify / Rf * sin(epsilon)) * vx;
    h53 = t;
    h54 = 0;
    h55 = K;
    h56 = -Ify / Rf * cos(epsilon) * vx;
    
    % Costruzione matrice H
    H = [
        h11, h12, h13, h14, h15, h16;
        h21, h22, h23, h24, h25, h26;
        h31, h32, h33, h34, h35, h36;
        h41, h42, h43, h44, h45, h46;
        h51, h52, h53, h54, h55, h56;
        h61, h62, h63, h64, h65, h66
    ];
    
    %% MATRICE P
    
    % riga 1
    p11 = 0;
    p12 = 0;
    
    % riga 2
    p21 = 0;
    p22 = 0;
    
    % riga 3
    p31 = -(Cf1 * cos(epsilon) + Cf2 * sin(epsilon));
    p32 = -Cf2;
    
    % riga 4
    p41 = 0;
    p42 = -Cr2;

    % riga 5
    p51 = (t * Zf - Mf * e * g) * sin(epsilon);
    p52 = t * Zf - Mf * e * g;

    % riga 6
    p61 = t * Zf - Mf * e * g;
    p62 = -(Mf * j + Mr * h) * g;
    
    % Costruzione matrice P
    P_old = [
        p11, p12;
        p21, p22;
        p31, p32;
        p41, p42;
        p51, p52;
        p61, p62
    ];
    
    %% VETTORE G
    G = [0; 0; 0; 0; -1; 0];
    
    %% MATRICE P
    P = [zeros(6,4) P_old];
    
    %% RISCRITTURUA STATE SPACE METODO 1
    O = zeros(6,6);
    I = eye(6);
    
    A21 = -M\P;
    A22 = -M\H;
    
    A = [O,     I;
         A21, A22];
end

