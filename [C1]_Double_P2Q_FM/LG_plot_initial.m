function [phandles] = LG_plot_initial(fig,state)
%LG_PLOT_INITIAL Summary of this function goes here
%   Detailed explanation goes here
%% 导入数据
xP  = state(1);
yP  = state(3);
zP  = state(5);

alpha1 = state(7);
beta1 = state(9);

phi1    = state(11);
theta1  = state(13);
psi1    = state(15);

alpha2 = state(17);
beta2 = state(19);

phi2    = state(21);
theta2  = state(23);
psi2    = state(25);

%%% 计算无人机1和无人机2的位置：
rho1 = [cos(beta1)*cos(alpha1),cos(beta1)*sin(alpha1),sin(beta1)];
rho2 = [cos(beta2)*cos(alpha2),cos(beta2)*sin(alpha2),sin(beta2)];
Lr = 1;

tmp1 = [xP,yP,zP]+Lr*rho1;
xQ1 = tmp1(1);
yQ1 = tmp1(2);
zQ1 = tmp1(3);

tmp2 = [xP,yP,zP]+Lr*rho2;
xQ2 = tmp2(1);
yQ2 = tmp2(2);
zQ2 = tmp2(3);

%% 数据预处理
%%% 无人机1姿态：旋转矩阵
R1 = euler2rotMat(phi1, theta1, psi1);

ux1 = R1(1,1);% each column vector of R is a direction vector
vx1 = R1(2,1);
wx1 = R1(3,1);
uy1 = R1(1,2);
vy1 = R1(2,2);
wy1 = R1(3,2);
uz1 = R1(1,3);
vz1 = R1(2,3);
wz1 = R1(3,3); 

R2 = euler2rotMat(phi2, theta2, psi2);
%%% 无人机2姿态：旋转矩阵
ux2 = R2(1,1);% each column vector of R is a direction vector
vx2 = R2(2,1);
wx2 = R2(3,1);
uy2 = R2(1,2);
vy2 = R2(2,2);
wy2 = R2(3,2);
uz2 = R2(1,3);
vz2 = R2(2,3);
wz2 = R2(3,3); 
%% 初始化figure属性
position = [50 50 866 600];
Xlabel = 'X(m)';
Ylabel = 'Y(m)';
Zlabel = 'Z(m)';
set(fig, 'Position', position);
set(gca, 'drawmode', 'fast');
lighting phong;
set(gcf, 'Renderer', 'zbuffer');
hold on;
axis equal;
grid on;
az = 120;
el = 20;
view(az, el);
xlabel(Xlabel,'Interpreter','LaTex');
ylabel(Ylabel,'Interpreter','LaTex');
zlabel(Zlabel,'Interpreter','LaTex');
%% 初始化 机体1坐标系句柄：
ratio = 0.4;
quivXhandleA = quiver3(xQ1, yQ1, zQ1, ratio*ux1, ratio*vx1, ratio*wx1,  'r','MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivYhandleA = quiver3(xQ1, yQ1, zQ1, ratio*uy1, ratio*vy1, ratio*wy1,  'g','MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivZhandleA = quiver3(xQ1, yQ1, zQ1, ratio*uz1, ratio*vz1, ratio*wz1,  'b','MaxHeadSize', 0.999999, 'AutoScale', 'off');
%% 初始化 机体2坐标系句柄：
ratio = 0.4;
quivXhandleB = quiver3(xQ2, yQ2, zQ2, ratio*ux2, ratio*vx2, ratio*wx2,  'r','MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivYhandleB = quiver3(xQ2, yQ2, zQ2, ratio*uy2, ratio*vy2, ratio*wy2,  'g','MaxHeadSize', 0.999999, 'AutoScale', 'off');
quivZhandleB = quiver3(xQ2, yQ2, zQ2, ratio*uz2, ratio*vz2, ratio*wz2,  'b','MaxHeadSize', 0.999999, 'AutoScale', 'off');
%% 初始化 四旋翼1句柄：
    len = 0.34; % distance from the center of rotor 1 to rotor 3; 
    x1A = [xQ1,yQ1,zQ1] + 0.5*len*[ux1,vx1,wx1];
    x3A = [xQ1,yQ1,zQ1] - 0.5*len*[ux1,vx1,wx1];
    x2A = [xQ1,yQ1,zQ1] + 0.5*len*[uy1,vy1,wy1];
    x4A = [xQ1,yQ1,zQ1] - 0.5*len*[uy1,vy1,wy1];
    QX1A = [x1A(1),x3A(1)];
    QX2A = [x1A(2),x3A(2)];
    QX3A = [x1A(3),x3A(3)];
    QY1A = [x2A(1),x4A(1)];
    QY2A = [x2A(2),x4A(2)];
    QY3A = [x2A(3),x4A(3)];
    
    
    eb1A = [ux1,vx1,wx1];
    eb2A = [uy1,vy1,wy1];
    eb3A = [uz1,vz1,wz1];
    temp = 0:(2*pi)/15 :2*pi;
    r0 = 0.04;
    xr1A = r0*cos(temp')*eb1A + r0*sin(temp')*eb2A + ones(size(temp'))*x1A;
    xr2A = r0*cos(temp')*eb1A + r0*sin(temp')*eb2A + ones(size(temp'))*x2A;
    xr3A = r0*cos(temp')*eb1A + r0*sin(temp')*eb2A + ones(size(temp'))*x3A;
    xr4A = r0*cos(temp')*eb1A + r0*sin(temp')*eb2A + ones(size(temp'))*x4A;

    lineQXhandleA = line('XData',QX1A,'YData',QX2A,'ZData',QX3A,'LineWidth',2,'Color',[1 0 0]);
    lineQYhandleA = line('XData',QY1A,'YData',QY2A,'ZData',QY3A,'LineWidth',2,'Color',[1 0 0]);
    circle1handleA = line('XData',xr1A(:,1),...
        'YData',xr1A(:,2),...
        'ZData',xr1A(:,3),...
        'LineWidth',2,'Color',[1 0 0]);
    
    circle2handleA = line('XData',xr2A(:,1),...
    'YData',xr2A(:,2),...
    'ZData',xr2A(:,3),...
    'LineWidth',2,'Color',[1 0 0]);

    circle3handleA = line('XData',xr3A(:,1),...
    'YData',xr3A(:,2),...
    'ZData',xr3A(:,3),...
    'LineWidth',2,'Color',[1 0 0]);

    circle4handleA = line('XData',xr4A(:,1),...
    'YData',xr4A(:,2),...
    'ZData',xr4A(:,3),...
    'LineWidth',2,'Color',[1 0 0]);
%% 初始化 四旋翼2句柄：
    len = 0.34; % distance from the center of rotor 1 to rotor 3; 
    x1B = [xQ2,yQ2,zQ2] + 0.5*len*[ux2,vx2,wx2];
    x3B = [xQ2,yQ2,zQ2] - 0.5*len*[ux2,vx2,wx2];
    x2B = [xQ2,yQ2,zQ2] + 0.5*len*[uy2,vy2,wy2];
    x4B = [xQ2,yQ2,zQ2] - 0.5*len*[uy2,vy2,wy2];
    QX1B = [x1B(1),x3B(1)];
    QX2B = [x1B(2),x3B(2)];
    QX3B = [x1B(3),x3B(3)];
    QY1B = [x2B(1),x4B(1)];
    QY2B = [x2B(2),x4B(2)];
    QY3B = [x2B(3),x4B(3)];
    
    
    eb1B = [ux2,vx2,wx2];
    eb2B = [uy2,vy2,wy2];
    eb3B = [uz2,vz2,wz2];
    temp = 0:(2*pi)/15 :2*pi;
    r0 = 0.04;
    xr1B = r0*cos(temp')*eb1B + r0*sin(temp')*eb2B + ones(size(temp'))*x1B;
    xr2B = r0*cos(temp')*eb1B + r0*sin(temp')*eb2B + ones(size(temp'))*x2B;
    xr3B = r0*cos(temp')*eb1B + r0*sin(temp')*eb2B + ones(size(temp'))*x3B;
    xr4B = r0*cos(temp')*eb1B + r0*sin(temp')*eb2B + ones(size(temp'))*x4B;

    lineQXhandleB = line('XData',QX1B,'YData',QX2B,'ZData',QX3B,'LineWidth',2,'Color',[1 0 0]);
    lineQYhandleB = line('XData',QY1B,'YData',QY2B,'ZData',QY3B,'LineWidth',2,'Color',[1 0 0]);
    circle1handleB = line('XData',xr1B(:,1),...
        'YData',xr1B(:,2),...
        'ZData',xr1B(:,3),...
        'LineWidth',2,'Color',[1 0 0]);
    
    circle2handleB = line('XData',xr2B(:,1),...
    'YData',xr2B(:,2),...
    'ZData',xr2B(:,3),...
    'LineWidth',2,'Color',[1 0 0]);

    circle3handleB = line('XData',xr3B(:,1),...
    'YData',xr3B(:,2),...
    'ZData',xr3B(:,3),...
    'LineWidth',2,'Color',[1 0 0]);

    circle4handleB = line('XData',xr4B(:,1),...
    'YData',xr4B(:,2),...
    'ZData',xr4B(:,3),...
    'LineWidth',2,'Color',[1 0 0]);
%% 初始化 payload 句柄
[sx,sy,sz] = sphere;
SPx = 0.05*sx + xP;
SPy = 0.05*sy + yP;
SPz = 0.05*sz + zP;
payloadhandle = surf(SPx,SPy,SPz,'FaceColor',[1 0 0],'EdgeColor','none');
%% 初始化 cable1 句柄
cablehandleA = line('Xdata',[xQ1,xP],...
        'Ydata',[yQ1,yP],...
        'Zdata',[zQ1,zP],...
        'LineWidth',2,'Color','k');
%% 初始化 cable2 句柄
cablehandleB = line('Xdata',[xQ2,xP],...
        'Ydata',[yQ2,yP],...
        'Zdata',[zQ2,zP],...
        'LineWidth',2,'Color','k');
%% 设置初始范围
LimitRatio = 1;AxisLength = 1.3;
Xlim = [xP-AxisLength xP+AxisLength] * LimitRatio;
Ylim = [yP-AxisLength yP+AxisLength] * LimitRatio;
Zlim = [zP-AxisLength zP+AxisLength] * LimitRatio;
set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim);
%% 返回句柄结构体
phandles.quivXhandleA = quivXhandleA;
phandles.quivYhandleA = quivYhandleA;
phandles.quivZhandleA = quivZhandleA;
phandles.quivXhandleB = quivXhandleB;
phandles.quivYhandleB = quivYhandleB;
phandles.quivZhandleB = quivZhandleB;

phandles.lineQXhandleA = lineQXhandleA;
phandles.lineQYhandleA = lineQYhandleA;
phandles.lineQXhandleB = lineQXhandleB;
phandles.lineQYhandleB = lineQYhandleB;

phandles.circle1handleA = circle1handleA;
phandles.circle2handleA = circle2handleA;
phandles.circle3handleA = circle3handleA;
phandles.circle4handleA = circle4handleA;
phandles.circle1handleB = circle1handleB;
phandles.circle2handleB = circle2handleB;
phandles.circle3handleB = circle3handleB;
phandles.circle4handleB = circle4handleB;

phandles.payloadhandle = payloadhandle;

phandles.cablehandleA = cablehandleA;
phandles.cablehandleB = cablehandleB;
phandles.Xlim = Xlim;
phandles.Ylim = Ylim;
phandles.Zlim = Zlim;
end

function R = euler2rotMat(phi, theta, psi)
    R(1,1,:) = cos(psi).*cos(theta);
    R(1,2,:) = -sin(psi).*cos(phi) + cos(psi).*sin(theta).*sin(phi);
    R(1,3,:) = sin(psi).*sin(phi) + cos(psi).*sin(theta).*cos(phi);
    
    R(2,1,:) = sin(psi).*cos(theta);
    R(2,2,:) = cos(psi).*cos(phi) + sin(psi).*sin(theta).*sin(phi);
    R(2,3,:) = -cos(psi).*sin(phi) + sin(psi).*sin(theta).*cos(phi);
    
    R(3,1,:) = -sin(theta);
    R(3,2,:) = cos(theta).*sin(phi);
    R(3,3,:) = cos(theta).*cos(phi);
end