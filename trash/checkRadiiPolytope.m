% deprecated
% this is to find the relation between the formulation of D and the center
% of the polyhedral set, and the way of formulating the center to the
% vertex

[lb, ub] = deal(-2,4);
figure(1),clf
% D
A = [0 -1; -1 -1; -1 1; 0 1; 1 1; 1 -1];
b = [0 ; -1; 1; 2; 4; 2];

Al = [A -ones(size(A, 1), 1)];

[sol, fval, exitflag] = linprog([0 0 1]', Al, b);

draw_face = true;
convex_region = true;
MAX_z = 0;
[x, y] = illustrate2DDomain(A, b, lb, ub, convex_region, MAX_z, [192,192,192]/255, draw_face); 

plot3(sol(1), sol(2), sol(3), '*k')