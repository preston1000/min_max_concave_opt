% main entrance of all the solvers within considerationi

%% load problem
index = 1;
problem = getProblemMaxOpt( index );

%% the proposed algorithm

[ result_my ] = useMy( problem );
if isempty(result_my) || result_my.exitflag ~= 1
    fprintf('Fail to solve using the proposed method.\n')
    x_my = [];
    obj_my = [];
    constr_value_my = [];
else
    x_my = result_my.x;
    obj_my = max(problem.obj.A * x_my + problem.obj.b);
    constr_value_my = zeros(length(problem.constraint), 1);
    for i = 1:length(problem.constraint)
        constr_value_my(i) = max(problem.constraint(i).A * x_my  + problem.constraint(i).b);
    end
end

%% use MILP, intlinprog, fval_milp, exitflag_milp 

[ x_milp] = useMILP( problem.obj, problem.constraint, ...
    problem.A, problem.b, problem.Aeq, problem.beq, problem.lb, problem.ub, problem.options );
if isempty(x_milp)
    fprintf('Fail to solve using the MILP.\n')
    obj_milp = [];
    constr_value_milp = [];
else
    obj_milp = max(problem.obj.A * x_milp + problem.obj.b);
    constr_value_milp = zeros(length(problem.constraint), 1);
    for i = 1:length(problem.constraint)
        constr_value_milp(i) = max(problem.constraint(i).A * x_milp  + problem.constraint(i).b);
    end
end

%% use MILP, Gurobi 

[ x_gurobi] = useGurobi( problem );
if isempty(x_gurobi)
    fprintf('Fail to solve using Gurobi.\n')
    obj_gurobi = [];
    constr_value_gurobi = [];
else
    obj_gurobi = max(problem.obj.A * x_gurobi + problem.obj.b);
    constr_value_gurobi = zeros(length(problem.constraint), 1);
    for i = 1:length(problem.constraint)
        constr_value_gurobi(i) = max(problem.constraint(i).A * x_gurobi  + problem.constraint(i).b);
    end
end

%%
[ x_grid ] = useGrid( problem.obj, problem.constraint, ...
    problem.A, problem.b, problem.Aeq, problem.beq, problem.lb, problem.ub, problem.options  );
obj_grid = max(problem.obj.A * x_grid + problem.obj.b);









