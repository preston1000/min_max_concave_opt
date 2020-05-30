function [ result ] = useMy( problem )
%USEMY my method
%   this is only valid for the simplest problem #1, no consideratio of more
%   complex situations.
%% parameters
if nargin < 1
    index = 1;
    problem = getProblemMaxOpt( index );
end
epsilon = 1e-6; % for strict comparison
use_line_search = false;

result = [];
%% model parameters
n_x = problem.n;

lb = min(problem.lb);
ub = max(problem.ub);

A = problem.A;
b = problem.b;

A_o = problem.obj.A;
b_o = problem.obj.b;

A_1 = problem.constraint(1).A;
b_1 = problem.constraint(1).b;
%% find optimum of min max(A_o x - b_o), s.t. Ax <= b
Al = [A zeros(size(A, 1), 1); A_o -ones(size(A_o, 1), 1)];
bl = [-b; -b_o];

[sol, ~, exitflag] = linprog([zeros(n_x, 1); 1], Al, bl, [], [], [lb; -inf], [ub; inf]);

if exitflag~=1
    return 
end
sol = sol(1:n_x);

X = zeros(n_x, 100);
X(:, 1) = sol;
iter = 2;
%% find a  direction to feasible domain
while(1)
    [violation, index_on_1] = findOnPiece( A_1, b_1, sol ); % only one

    if violation >= 0 % the optimum is found without further action.
        if iter == 2
            fprintf('The optimum is found without consideration of maximum constraints.\n')
            result = struct('x', sol, 'exitflag', 1);
            return
        else
            fprintf('enter the feasible domain.\n')
            break
        end
    end
    A_dir = [   -A_1(index_on_1(1), :) 0; 
                    A zeros(size(A, 1), 1);
                    A_o -ones(size(A_o, 1), 1)];
    b_dir = [(-epsilon+violation); 
                (-b - A*sol);
                (-b_o - A_o * sol)];

    [dir, ~, exitflag] = linprog([zeros(n_x, 1); 1], A_dir, b_dir, [], [], [lb; -inf], [ub; inf]);

    if exitflag == 1
        sol_2 = sol + dir(1:2);
    else
        fprintf('no feasible direction is found')
        result = struct('x', sol, 'exitflag', -1);
        return
    end
    X(:, iter) = sol_2;
    sol = sol_2;
    iter = iter + 1;
end

%% once get in the feasible domain, add a constraint to D 
[~, index_on] = findOnPiece( A_1, b_1, sol ); % only one

A_dir = [-A_1(index_on(1), :) 0; 
            A zeros(size(A, 1), 1);
            A_o -ones(size(A_o, 1), 1)];
b_dir = [b_1(index_on(1)); 
            -b ;
            -b_o];
[sol, ~] = linprog([zeros(n_x, 1); 1], A_dir, b_dir, [], [], [lb; -inf], [ub; inf]);
X(:, iter) = sol(1:n_x);
X = X(:, 1:iter);

result = struct('x', sol, 'X', X, 'exitflag', 1);

end

