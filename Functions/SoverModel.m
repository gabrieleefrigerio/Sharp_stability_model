function [Results] = SoverModel(Bike)
% ----------------- Primo calcolo per identificare n_modi -----------------
Results = struct();
Results.A = Sharp_Model(Bike,1);
[~, D] = eig(Results.A);

Results.n_modi = size(D, 1);

% ----------------- Preallocazioni -----------------
Results.frequenze_naturali = zeros(Results.n_modi, Bike.Input.n_points);
Results.smorzamenti = zeros(Results.n_modi, Bike.Input.n_points);
Results.stabilita = strings(Results.n_modi, Bike.Input.n_points);

Results.Capsize = zeros(1, Bike.Input.n_points);
Results.Weave   = zeros(1, Bike.Input.n_points);
Results.Wobble  = zeros(1, Bike.Input.n_points);

Results.A = zeros(Results.n_modi, Results.n_modi, Bike.Input.n_points);

% ----------------- Calcoli dinamici -----------------
for i = 1:Bike.Input.n_points
    Results.A(:,:,i) = Sharp_Model(Bike, i);
end

[Results.V,Results.D] = eigenshuffle(Results.A);


for i = 1:Bike.Input.n_points
    for j = 1:Results.n_modi
        lambda = Results.D(j,i);
        sigma = real(lambda);
        omega = abs(imag(lambda));
        omega_n = abs(lambda);
        zeta = (omega_n == 0) * 1 + (omega_n ~= 0) * (-sigma / omega_n);
        freq_n = omega / (2 * pi);
    
        Results.frequenze_naturali(j, i) = freq_n;
        Results.smorzamenti(j, i) = zeta;
        if sigma < 0
            stabilita(j, i) = "Si";
        else
            stabilita(j, i) = "No";
        end

    end
end

end

