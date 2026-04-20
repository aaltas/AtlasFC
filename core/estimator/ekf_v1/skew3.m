% =========================================================================
%  SKEW3 - 3x3 skew-symmetric (cross-product) matrix
% =========================================================================
%  Returns the 3x3 skew-symmetric matrix [v]× such that:
%    [v]× * u  =  cross(v, u)
%
%  Input : v [3x1]
%  Output: S [3x3]  skew-symmetric
%
%  Convention:
%    S = [  0   -v3   v2 ]
%        [  v3   0   -v1 ]
%        [ -v2   v1   0  ]
%
%  Author : AtlasFC
% =========================================================================

function S = skew3(v)

    S = [  0,    -v(3),  v(2);
           v(3),  0,    -v(1);
          -v(2),  v(1),  0   ];

end
