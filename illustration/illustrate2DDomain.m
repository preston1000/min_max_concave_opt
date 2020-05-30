function [x, y, z] = illustrate2DDomain(A, b, lb, ub, convex, MAX, color, draw_face)
% this is to plot the feasible domain bounded by Ax <=b
% 
% convex: 1(max(A x + b) <= MAX), 0(max(A x + b) >= MAX)
% color: the color of contour or face
% draw_face: 1(draw face), 0(draw contour)
% lb, ub: when scalar, it;s the lower / upper bound of variables,
%       otherwise, it's the given points to be evaluated
    if isscalar(lb) 
        step = (ub-lb)/500;
        [x, y] = meshgrid(lb:step:ub);
        xlim = lb;
        xlim2 = ub;
        ylim = lb;
        ylim2 = ub;
    else
        x = lb;
        y = ub;
        xlim = min(min(x));
        xlim = xlim - 0*abs(xlim);
        xlim2 = max(max(x));
        xlim2 = xlim2 + 0*abs(xlim2);
        ylim = min(min(y));
        ylim = ylim - 0*abs(ylim);
        ylim2 = max(max(y));
        ylim2 = ylim2 + 0*abs(ylim2);
    end
    
    z = zeros(size(x));
    for i = 1:size(x, 1)
        for j = 1:size(x, 2)
            tmp = max(A * [x(i, j); y(i, j)] - b);
            if convex
                if tmp <= MAX
                    z(i, j) = tmp;
                else
                    z(i, j) = nan;
                end
            else
                if tmp <= MAX
                    z(i, j) = nan;
                else
                    z(i, j) = tmp;
                end
            end
        end
    end
    if draw_face
        mesh(x, y, z,'FaceColor',color,'FaceLighting','none','EdgeColor','none','FaceAlpha', 0.5)
    else
        contour3(x, y, z,[-1 -0.8 -0.6 -0.4 -0.2 0 1:8],color)
    end
    hold on
    
    axis([xlim, xlim2, ylim, ylim2])
    axis square
end