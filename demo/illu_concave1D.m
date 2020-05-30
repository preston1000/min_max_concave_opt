% this is to check the shape of max(a_i x + b_) >= 0， 1D

%% example 1: the whole range, -infty, infty
% x = -20:.1:20;
% f1 = 3*x + 1;
% f2 = 0*x + 5;
% f3 = -2*x +1; 
% f = max([f1;f2;f3]);
% figure(2), clf
% plot(x, f)
% plotShadow(1, x, f)
%% example 2: single function, concave
% x = -20:.1:20;
% f1 = 3*x + 1;
% f2 = 0*x + 5;
% f3 = -2*x +1; 
% f = max([f1;f2;f3])-10;
% plotShadow(2, x, f)

%% example 3: two functions, with intersection in the feasible domains respectively
% x = -20:.1:20;
% f1 = 3*x + 1;
% f2 = 0*x + 5;
% f3 = -2*x +1; 
% f4 = -2*x -6;
% f5 = 3*x +6;
% f = max([f1;f2;f3])-10 +max([f4;f5])-10 ;
% figure(3), clf
% plot(x, f)
% plotShadow(3, x, f)

%% example 3: two functions, with intersection in the feasible domains respectively
x = -30:.1:20;
f1 = 3*x + 1;
f2 = 0*x + 5;
f3 = -2*x +1; 
f4 = -1.5*x -26;
f5 = 0.3*x +16;
f = max([f1;f2;f3])-10  ;
ff = +max([f4;f5])-10 ;
% f = 0.1*f + ff;
figure(4), clf
plotShadow(4, x, f)

plotShadow(4, x, ff)

%%
function segments=plotShadow(figID,x, y)
    x = x(:);
    y = y(:);
    segments = zeros(0, 2);
    start = -1;
    for i = 1:length(y)
        if y(i) >=0
            start = i;
            finish = i;
            break
        end
    end
    if start == -1
        return
    end
        
    count = 1;
    while(finish <= length(y) && start <= length(y))
        if finish < 0
            if y(start) >= 0
                finish = start;
            else
                start =start + 1;
                
            end
        else
            if y(finish) <0
                segments(count, :) = [start, finish-1];
                count = count + 1;
                start = finish;
                finish = -1;
            else
                finish = finish + 1;
            end
        end
    end
    num_seg = size(segments,1);
    if finish > 0 && start > 0
        if (num_seg==0) || segments(num_seg, 1) ~= start
            segments(count, :) = [start, length(y)];
        end
    end
    
    figure(figID)
    for i = 1:size(segments, 1)
        seg_y = y(segments(i, 1):segments(i, 2));
        seg_x = x(segments(i, 1):segments(i, 2));
        
        ver_x = [seg_x;flipud(seg_x)];
        ver_y = [seg_y-0.5;flipud(seg_y)+0.5];
        
        fill(ver_x, ver_y,[96 96 96]/255)
        hold on
    end
    
    plot(x, y,'green')
end