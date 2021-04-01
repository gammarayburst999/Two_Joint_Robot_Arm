% developing 2 link robotic arm
mdl_planar2
[x,z] = meshgrid(-10:1:10);
y1=zeros(length(x)); % first obstacle
y2=0.6*ones(length(x)); % second obstacle
%plot
surf(x, y1, z, 'EdgeColor', 'none')
hold on
surf(x, y2, z, 'EdgeColor', 'none')
p2.plot(qz)
xlabel('x')
ylabel('y')
zlabel('z')