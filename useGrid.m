function [ sol, fval ] = useGrid( obj, constraint, A, b, Aeq, beq, lb, ub, options  )
%GRIDSEARCHOPTIMUM grid search for optimum
% ˜Ž

    n_x = size(obj.A, 2);
    if isscalar(lb)
        lb = lb * ones(n_x, 1);
    end
    if isscalar(ub)
        ub = ub * ones(n_x, 1);
    end

    x = lb(1):0.01:ub(1);
    y = lb(2):0.01:ub(2);
    [x, y] = meshgrid(x, y);
    
    % objective function value
    z_obj = nan(size(x));
    for i = 1:size(x, 1)
        for j = 1:size(x, 2)
            sol = [x(i, j); y(i, j)];
            if x(i, j)== -2 &&  y(i, j) == -2
                i;
            end
            if any(A * sol + b > 0)
                continue
            end
            flag = false;
            for k = 1:length(constraint)
                tmp_value = max(constraint(k).A * sol + constraint(k).b);
                if tmp_value < 0
                    flag = true;
                    break
                end
            end
            if flag
                continue
            end
            
            tmp = max(obj.A * sol + obj.b);
            z_obj(i, j) = tmp;
        end
    end
    
    tmpp = z_obj(:);
    tmpp(isnan(tmpp)) = 1000;
    [~, index] = min(tmpp);
    [i, j] = ind2sub(size(x), index);
    sol = [x(i,j); y(i,j)];
    fval = z_obj(i,j);
    fprintf('best solution: <%s>, value: %f \n', num2str(sol'), fval)
end

