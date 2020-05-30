% this is to illustrate the kmax function in 2D (neither convex nor concave)


n = 2;
M = 10;
k = 3;

a = rand(M, n)-0.5;
b = rand(M,1)-0.5;

[x, y] = meshgrid(-10:0.01:10);

z = zeros(size(x));

for i = 1:size(x, 1)
    for j = 1:size(x, 2)
        tmp = a*[x(i,j);y(i,j)] + b;
        tmp = sort(tmp, 'descend');
        z(i,j) = tmp(k);
    end
end

figure(1)
clf
mesh(x, y, z)
hold on
contour(x, y, z)