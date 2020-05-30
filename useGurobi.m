function [x] = useGurobi(problem)
%USEGUROBI Summary of this function goes here
%   solve the max constrained problem using Gurobi.
%  formulatioin is based on the built-in genconmax.

    %%
    if nargin < 1
        index = 1;
        problem = getProblemMaxOpt( index );
    end
    model = struct();
    %%
    n_x = problem.n;
    n_z = 1;
    n_y = length(problem.constraint);
    n_w_sep = zeros(n_y, 1);
    for i = 1:n_y
        n_w_sep = size(problem.constraint(i).A, 1);
    end
    n_w = sum(n_w_sep);
    %%
    if isempty(problem.lb)
        lb = [-inf(n_x + n_z, 1); zeros(n_y, 1); -inf(n_w, 1)];
    else
        if isscalar(problem.lb)
            lb = problem.lb * ones(n_x, 1);
        else
            lb = problem.lb;
        end
        lb = [lb; -inf; zeros(n_y, 1); -inf(n_w, 1)];
    end
    model.lb = lb;
    %%
    if isempty(problem.ub)
        ub = inf(n_x + n_z + n_y + n_w, 1);
    else
        if isscalar(problem.ub)
            ub = problem.ub * ones(n_x, 1);
        else
            ub = problem.ub;
        end
        ub = [ub; inf(n_z + n_y + n_w, 1)];
    end
    model.ub = ub;
    %%
    A = [problem.A zeros(size(problem.A, 1), n_z + n_y + n_w);% A x <= b
        problem.obj.A -ones(size(problem.obj.A, 1), 1) zeros(size(problem.obj.A, 1), n_y + n_w); % A_0 x + b_0 <= z
        ]; %
    b = [-problem.b;
         -problem.obj.b];
     
    Aeq = [problem.Aeq zeros(size(problem.Aeq, 1), n_z + n_y + n_w);];
    beq = -problem.beq;
    
    for i = 1:n_y
        n_row = size(problem.constraint(i).A, 1);
        tmp_A = [problem.constraint(i).A zeros(n_row, n_y + n_z) zeros(n_row, sum(n_w_sep(1:(i-1)))) -eye(n_w_sep(i)) zeros(n_row, sum(n_w_sep((i+1):end)))];
        tmp_b = -problem.constraint(i).b;
        
        Aeq = [Aeq; tmp_A];
        beq = [beq; tmp_b];
    end
    
    model.sense = repelem('<=', [length(b), length(beq)]);
    model.A = sparse([A; Aeq]);
    model.rhs = [b; beq];
    
    %% 
    genconmax = struct();
    for i = 1:n_y
        genconmax(i).resvar = n_x + n_z + i;
        genconmax(i).vars = n_x + n_z + n_y + sum(n_w_sep(1:(i-1))) + (1:n_w_sep(i));
        genconmax(i).name = ['y' num2str(i)];
    end
    model.genconmax = genconmax;
    model.modelsense = 'min';
    model.obj = zeros(n_x + n_z + n_y + n_w, 1);
    model.obj(n_x + 1) = 1;
    problem.vtype = repelem('C', n_x + n_z + n_y + n_w);
    %%
    result = gurobi(model);
    if strcmp(result.status, 'OPTIMAL')
        x = result.x(1:n_x);
    else
        x = [];
    end
end

