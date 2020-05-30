function [sol_4] = illu_concave2D()
% this function is the first version, which solves the simplest situation
% in 2D, with illustration 

%clear, clc, 
figure(1),clf

index = 1;
problem = getProblemMaxOpt( index );

lb = min(problem.lb);
ub = max(problem.ub);

epsilon = 1e-6; % for strict comparison
%% D, A x <=b
% A = [0 -1; -1 -1; -1 1; 0 1; 1 1; 1 -1];
% b = [0 ; -1; 1; 2; 4; 2];
A = problem.A;
b = problem.b;

draw_face = true;
convex_region = true;
MAX_z = 0;
[x, y] = illustrate2DDomain(A, -b, lb, ub, convex_region, MAX_z, [192,192,192]/255, draw_face); 

%% objective, max(A_o * x - b_o)
% A_o = [2 1; -1 3; 0 -1; -3 -2];
% b_o = [5;4.5;-0.5;-4.5];
A_o = problem.obj.A;
b_o = problem.obj.b;

draw_face = false;
convex_region = true;
MAX_z = 100;
[xx, yy,zz] = illustrate2DDomain(A_o, -b_o, x, y, convex_region, MAX_z,'r--', draw_face) ;

%% find optimum of min max(A_o x - b_o), s.t. Ax <= b
Al = [A zeros(size(A, 1), 1); A_o -ones(size(A_o, 1), 1)];
bl = [-b; -b_o];

[sol, fval, exitflag] = linprog([0 0 1]', Al, bl);

if exitflag~=1
    return
end
plot3(sol(1), sol(2), sol(3), '*k')

%% design D_1 = {x : f_1(x) = max(A_1 x - b_1) >= 0} and illustrate (x^\star lies outside D_1)
% A_1 = [-4 -5; -1 -13; 5 -7];
% b_1 = [-7.5; -14.8; 2];
% scale = sqrt((sum([A_1 b_1].^2, 2)));
% A_1 = A_1./scale;
% b_1 = b_1./scale;
A_1 = problem.constraint.A;
b_1 = problem.constraint.b;

draw_face = true;
convex_region = false;
MAX_z = 0;

illustrate2DDomain(A_1, -b_1, x, y, convex_region, MAX_z, [0,191,255]/255, draw_face) ;

violation = max(A_1 * sol(1:2) - b_1);
%% find a  direction to feasible domain --- the direct application of the update leads to an infeasible solution
[~,index_on_1] = findOnPiece( A_1, b_1, sol(1:2) ); % only one

A_dir = [-A_1(index_on_1(1), :) 0; A zeros(size(A, 1), 1);A_o -ones(size(A_o, 1), 1)];
b_dir = [(-epsilon+violation); (-b - A*sol(1:2));(-b_o - A_o * sol(1:2))];

[dir, ~, exitflag] = linprog([0 0 1]', A_dir, b_dir);

if exitflag == 1
    sol_2 = sol(1:2) + dir(1:2);
    plot3([sol(1) sol_2(1)], [sol(2) sol_2(2)], [0,0],'green-','linewidth',3)
    plot3(sol_2(1), sol_2(2), 0,'green->','linewidth',3)
end


violation = max(A_1 * sol_2(1:2) + b_1)

%% try line search and repeated update, to see which is valid
use_line_search = false;

if use_line_search
    
else
    [~, index_on_1] = findOnPiece( A_1, b_1, sol_2(1:2) ); % only one

    A_dir = [-A_1(index_on_1(1), :) 0; A zeros(size(A, 1), 1);A_o -ones(size(A_o, 1), 1)];
    b_dir = [(-epsilon+violation); (-b - A*sol(1:2));(-b_o - A_o * sol(1:2))];

    [dir_2, ~, exitflag_2] = linprog([0 0 1]', A_dir, b_dir);
    if exitflag_2 == 1
        sol_3 = sol_2(1:2) + dir_2(1:2);
        plot3([sol_2(1) sol_3(1)], [sol_2(2) sol_3(2)], [0,0],'green-','linewidth',3)
        plot3(sol_3(1), sol_3(2), 0,'redo','linewidth',3)
    end

    violation = max(A_1 * sol_3(1:2) + b_1)
end

%% once get in the feasible domain, add a constraint to D 
[~, index_on_2] = findOnPiece( A_1, b_1, sol_3(1:2) ); % only one

A_dir = [-A_1(index_on_2(1), :) 0; A zeros(size(A, 1), 1);A_o -ones(size(A_o, 1), 1)];
b_dir = [b_1(index_on_2(1)); -b ;-b_o];
[sol_4, ~, exitflag_2] = linprog([0 0 1]', A_dir, b_dir);


%%
view([0 0 1])
legend({'D', 'Obj', '$$x^{\star}$$', '$$D_{1}$$', 'v'},'Interpreter','latex')
