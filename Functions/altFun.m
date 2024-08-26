function D2 = altFun(ZI,ZJ)
% calculation of altitude

% Extract altitude
alt_vp = ZI(3); % altitude single observation
alt_vp_all = ZJ(:, 3); % altitude multiple observations

D2 = abs(alt_vp - alt_vp_all);

end