function [ problem ] = getProblemMaxOpt( index )
%GETPROBLEMMAXOPT test examples
%   
    if index == 1
        problem = number_1();
    else
        problem = [];
    end

end

function problem = number_1()
% 2D, polytope feasible domain, with only one max constraint, which exludes
% the optimum. The objective is a max function
    A = [0 -1; -1 -1; -1 1; 0 1; 1 1; 1 -1];
    b = -[0 ; -1; 1; 2; 4; 2]; % A x + b <= 0
    
    A_o = [2 1; -1 3; 0 -1; -3 -2];
    b_o = -[5;4.5;-0.5;-4.5];
    
    A_1 = [-4 -5; -1 -13; 5 -7];
    b_1 = -[-7.5; -14.8; 2];
    scale = sqrt((sum([A_1 b_1].^2, 2)));
%     A_1 = A_1./scale;
%     b_1 = b_1./scale;
    
    obj = struct('A', A_o, 'b', b_o);
    constraint = struct('A', A_1, 'b', b_1);
    
    problem = struct('n', 2, 'A', A, 'b', b, 'obj', obj, 'constraint', constraint, ...
        'Aeq', [], 'beq', [], 'lb', [-2;-2], 'ub', [4;4], 'options', []);
end
