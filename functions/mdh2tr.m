function [T, T_i] = mdh2tr(mdh)
% mdh2tr() - calculate transformation matrix from mdh parameter
%
% Inputs:
%    mdh (ix4 Matrix) - colums: alpha, a, d, delta [SI Units]
%
% Outputs:
%    T      - hom. transformation (4x4)
%    T_i    - all transformations ^{i-1}T_i
T = eye(4);
T_i = nan(4,4,size(mdh,1));
for k = 1:size(mdh,1)
    sa = sin(mdh(k,1));
    ca = cos(mdh(k,1));
    a = mdh(k,2);
    d = mdh(k,3);
    sd = sin(mdh(k,4));
    cd = cos(mdh(k,4));
    T_i(:,:,k) = [  cd, -sd, 0, a;...
                    sd*ca, cd*ca, -sa, -d*sa;...
                    sd*sa, cd*sa, ca, d*ca;...
                    0, 0, 0, 1];
    T = T*T_i(:,:,k);
end
end