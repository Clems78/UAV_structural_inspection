function [ex, ey, ez] = ellipsoidPoints(cx, cy, cz, rmaj, rmin, c_normal)
% Generates points for an ellipse in 3D space centered at (cx, cy, cz)
% with major axis length rmaj, minor axis length rmin, and normal vector 'normal'

theta = linspace(0, 2*pi, 100);
ellipse = [rmaj * cos(theta); rmin * sin(theta)];

% Find two orthogonal vectors in the plane of the normal vector
if all(c_normal == [0, 0, 1])
    % Special case where normal is [0, 0, 1]
    v1 = [1, 0, 0];
    v2 = [0, 1, 0];
else
    v1 = cross(c_normal, [0, 0, 1]);
    v1 = v1 / norm(v1);
    v2 = cross(c_normal, v1);
    v2 = v2 / norm(v2);
end

% Rotate the ellipse points to align with the normal vector
ex = cx + v1(1) * ellipse(1, :) + v2(1) * ellipse(2, :);
ey = cy + v1(2) * ellipse(1, :) + v2(2) * ellipse(2, :);
ez = cz + v1(3) * ellipse(1, :) + v2(3) * ellipse(2, :);
end