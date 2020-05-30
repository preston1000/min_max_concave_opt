function [ x ] = useMILP( obj, constraint, A, b, Aeq, beq, lb, ub, options )
%USEMILP formulate maximum optimization as MILP, ans use intlinprog/Gurobi to find
%the global optimum.
%   but the method seems to fail. try useGurobi.m for a different
%   formulation
% 
%   obj.A, obj.b: max(A x + b)
%   constraint: array, constraint(1).A, constraint(1).b -- max(A x + b) >= 0
%   A, b: linear inequality -- A x <= b
%   Aeq, beq: linear equality -- Aeq = beq
%   lb, ub: bounds
    
    M = 1000;
    n_constr = length(constraint);
    n_ys = zeros(n_constr, 1);
    for i = 1:n_constr
        n_ys = size(constraint(i).A, 1);
    end
    n_x = size(obj.A, 2) ;
    n_y = sum(n_ys);
    n_var = n_x + 1 + n_y ;
    
    Aeq = [Aeq  zeros(size(Aeq, 1))];
    for i = 1:n_constr
        tmp = zeros(1, n_var);
        tmp(n_x + 1+ sum(n_ys(1:(i-1))) + (1:n_ys(i))) = 1;
        Aeq = [Aeq; tmp];
    end
    problem.Aeq = Aeq;
    problem.beq = [beq; ones(n_constr, 1)];
    
    if isempty(lb)
        lb = -inf(n_x, 1);
    end
    if isempty(ub)
        lb = inf(n_x, 1);
    end
    problem.lb = [lb; -inf; zeros(n_y, 1)];
    problem.ub = [ub; inf; ones(n_y, 1)];
    
    problem.f = [zeros(n_x, 1) ;1 ;zeros(n_y, 1)];
    problem.intcon = (n_x+1+1):n_var;
    
    problem.solver = 'intlinprog';
    if isempty(options)
        problem.options = optimoptions('intlinprog');
    end
    
    tmp_A = [A zeros(size(A, 1), n_y+1); obj.A -ones(size(obj.A, 1), 1) zeros(size(obj.A, 1), n_y) ];
    tmp_b = [-b; -obj.b];
    
    total_row = sum(n_ys .* (n_ys - 1));
    [index_i, index_j, values] = deal([]);
    bs = [];
    count = 1;
    for i = 1:n_constr
        prev = sum(n_ys(1:(i-1)));
        for j = 1:(n_ys(i)-1)
            for k = (j+1):n_ys(i)
                tmp_col = [1:n_x, (n_x+1+prev+j) (n_x+1+prev+k) 1:n_x, (n_x+1+prev+k) (n_x+1+prev+j)];
                tmp_row = [(count * ones(1, n_x + 2)) ((count+1) * ones(1, n_x + 2))];
                tmp_values = [constraint(i).A(j, :)-constraint(i).A(k, :) M -M constraint(i).A(k, :)-constraint(i).A(j, :) M -M];
                index_i = [index_i tmp_row];
                index_j = [index_j tmp_col];
                values = [values tmp_values];
                bs = [bs; constraint(i).b(k)-constraint(i).b(j)+M; constraint(i).b(j)-constraint(i).b(k)+M];
                count = count + 2;
            end
        end
    end
    A_c = sparse(index_i, index_j, values,total_row, n_var);
    
    tmp_tmp_A = [];
    tmp_tmp_b = [];
    for i = 1:n_constr
        tmp_tmp_A = [tmp_tmp_A; -constraint(i).A zeros(n_ys(i), 1) zeros(n_ys(i), sum(n_ys(1:(i-1)))) M*eye(n_ys(i)) zeros(n_ys(i), n_y - sum(n_ys(1:i)))];
        tmp_tmp_b = [tmp_tmp_b; constraint(i).b+M];
    end
    tmp_A = [sparse(tmp_A) ;A_c; sparse(tmp_tmp_A)];
    tmp_b = [tmp_b; bs;tmp_tmp_b];
    
    
    
    problem.Aineq = tmp_A;
    problem.bineq = tmp_b;
    
    %[x ] = intlinprog(problem.f, problem.intcon, problem.Aineq, problem.bineq, ...
    %    problem.Aeq, problem.beq, problem.lb, problem.ub);
    problem = transform2GurobiFormat(problem.f, problem.Aineq, problem.bineq, ...
        problem.Aeq, problem.beq, problem.lb, problem.ub, n_x, n_y);
    result = gurobi(problem);
    if strcmp(result.status, 'OPTIMAL')
        x = result.x(1:n_x);
    else
        x = [];
    end
    printSolution(result, n_x)
end


function problem = transform2GurobiFormat(f, A, b, Aeq, beq, lb, ub, n_x, n_y)
    problem.A = [A; Aeq];
    problem.rhs = [b; beq];
    problem.lb = lb;
    problem.ub = ub;
    problem.obj = f;
    problem.sense = repelem('<=', [length(b), length(beq)]);
    
    problem.vtype = repelem('CB', [n_x+1, n_y]);
    problem.modelsense = 'min';
    
end

function printSolution(result, n_x)
    if strcmp(result.status, 'OPTIMAL')
        x       = result.x(1:n_x);
        value = result.x(1+n_x);
        fprintf('\nValue: %f\n', value);
        fprintf('\nSolution:\n')
        for f=1:n_x
            fprintf('%f, ', x(f));
        end
        fprintf('\n')
    else
        fprintf('No solution\n');
    end
end
