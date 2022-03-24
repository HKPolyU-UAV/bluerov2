% X: nxN
% C: rbf center(s) - nx1 or nxK, if C is n x K, then the result is STACKED
% over the centers K, i.e., of dimension K x N
% eps: kernel width for Gaussian type rbfs (optional)
% k: polyharmonic coefficient for polyharmonic rbfs (optional)
% Note that all RBFs are symmetrical so evaluation of of a single point on
% multiple centers can by done as evaluation of multiple points (the
% centers in this case) on a single point (the center)

function Y = rbf( X,C, type, eps, k )
type = lower(type);
if(~exist('eps','var') || isempty(eps))
    eps = 1;
end
if(~exist('k','var') || isempty(k))
    k = 1;
end

Cbig = C;
Y = zeros(size(C,2),size(X,2));
for i = 1:size(Cbig,2)
    C = Cbig(:,i);
    C = repmat( C,1, size(X,2) );
    r_squared = sum( (X - C).^2 );
    switch type
        case 'thinplate'
            y = r_squared.*log(sqrt(r_squared));  % + 0.5*sqrt(r_squared); % makes nonnegative, try to impose nonnegativity of K
            
            y(isnan(y)) = 0;
        case 'gauss'
            y = exp(-eps^2*r_squared);
        case 'invquad'
            y = 1 ./ (1+eps^2*r_squared);
        case 'invmultquad'
            
            y = 1 ./ sqrt((1+eps^2*r_squared));
        case 'polyharmonic'
            y = r_squared.^(k/2).*log(sqrt(r_squared));
            y(isnan(y)) = 0;
        otherwise
                error('RBF type not recognize')
    end
    Y(i,:) = y;
end
