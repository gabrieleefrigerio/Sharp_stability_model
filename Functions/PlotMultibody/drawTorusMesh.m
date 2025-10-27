function [P] = drawTorusMesh(N, R, r)

    % Parametri angolari
    theta = linspace(0, 2*pi, N);  % circonferenza grande
    phi   = linspace(0, 2*pi, N);  % sezione circolare
    [Tt, Pf] = meshgrid(theta, phi);

    % Equazioni parametriche
    X = (R - r + r .* cos(Pf)) .* cos(Tt);
    Y = r .* sin(Pf);
    Z = (R - r + r .* cos(Pf)) .* sin(Tt);

    % Costruisco i punti omogenei
    P = [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];
end

