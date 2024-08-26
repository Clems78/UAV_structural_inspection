function D2 = distFun(ZI, ZJ, rmaj, rmin, alpha_t)
    % ZI is a 1-by-n vector containing a single observation.
    % ZJ is an m2-by-n matrix containing multiple observations.
    % distfun must accept a matrix ZJ with an arbitrary number of observations.
    % D2 is an m2-by-1 vector of distances, and D2(k) is the distance between observations ZI and ZJ(k,:).

    % disp("Clustering..");

    % Extract locational and directional parts
    b_x = ZI(1:3);
    d_x = ZI(4:6);

    b_C = ZJ(:, 1:3);  % locational parts of ZJ
    d_C = ZJ(:, 4:6);  % directional parts of ZJ
    
    % Calculate the angle theta between bx-bC and dC
    b_diff = b_C - b_x;
    theta = acos(dot(b_diff, d_C, 2) ./ (vecnorm(b_diff, 2, 2) .* vecnorm(d_C, 2, 2)));
    
    e_b = vecnorm(b_x - b_C, 2, 2) .* sqrt((rmaj^2 / rmin^2) .* cos(theta).^2 + sin(theta).^2);

    % Compute directional distance (e_d)
    e_d = 1 - dot(repmat(d_x, size(d_C, 1), 1), d_C, 2);

    % Combine the distances
    % alpha_t = 0.1;  % Example threshold
    wp = 1;  % weight penalty
    
    w = ones(size(e_d));
    w(e_b > rmaj | e_d > 1 - cos(deg2rad(alpha_t))) = wp;

    c = (max(ZJ(:, 1)) + max(ZJ(:, 2)) + max(ZJ(:, 3)) + min(ZJ(:, 1)) + min(ZJ(:, 2)) + min(ZJ(:, 3))) / 6;

    D2 = w .* sqrt(e_b.^2 + (c * e_d).^2);
end
