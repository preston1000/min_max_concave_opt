function [ fun_value, index_on ] = findOnPiece( A, b, x )
%FINDONPIECE this function is to find which affine function the given point
%lies on
%   f = max(A x + b)
% fun_value: function value at the given x
% index_on: the indices of the affine function where the given x activates 

    tmp = A*x + b;
    fun_value = max(tmp);
    if nargout > 1
        index_on = find(tmp == fun_value);
    end
end

