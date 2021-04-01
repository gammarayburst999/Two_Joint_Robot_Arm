clear variables
close all;
time=3; %time given by user
len=0:0.1:time;
% developing 2 link robotic arm
import ETS2.*
a1 = 1; a2 =1;
%DH parameters
E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2);

T0=E.fkine( [0, 0], 'deg'); %forward Kinematic
Tf=E.fkine( [-47, 125], 'deg'); %
q0=T0.t; %start point of the end effector in Cartesian space
qf=Tf.t;  %end point of the end effector in Cartesian space
%joint space trajectory formed by quintic polynomials
[x, xd, xdd]=jtraj(q0,qf,len);
%calcuting the joint angles using inverse kinematic
q=zeros(length(x),2);
xv=x(:,1);
yv=x(:,2);
mdl_planar2
Tm=zeros(4,4,length(x));
for i=1:length(x)
    Tm(:,:,i)=transl(xv(i),yv(i),0);
    qq=p2.ikine( Tm(:,:,i),[0 0],'mask',[1 1 0 0 0 0]);
    if(isempty(qq))
        qq=[0 0];
    end
    q(i,1)=qq(1); %theata for arm 1 for all the x values
    q(i,2)=qq(2);  %theata for arm 2 for all the y values
end

%plot
figure(1)
plot(x(:,1),x(:,2))
hold on
grid on
scatter(q0(1),q0(2))
scatter(qf(1),qf(2),'d')
legend('locus','start point','end point')
title('The locus of the end effector in Cartesian space')
xlabel('joint1')
ylabel('joint2')
hold off

figure(2)
hold on
grid on
plot(len',xd(:,1),len',xd(:,2));
title('Velocity over time')
xlabel('time[s]')
ylabel('velocity[m/s]')
legend('joint 1','joint 2')
hold off

figure(3)
hold on
grid on
plot(len',xdd(:,1),len',xdd(:,2));
title('Acceleration over time')
xlabel('time[s]')
ylabel('Acceleration[m/s^2]')
legend('joint 1','joint 2')
hold off

figure(4)
hold on
grid on
plot(len',q(:,1),len',q(:,2));
title('Joint angles over time')
xlabel('time[s]')
ylabel('Angle in Radian')
legend('joint 1','joint 2')
hold off
figure(5)
E.plot( [0, 0], 'deg')
figure(6)
E.plot( [-47, 125], 'deg')