close all;
clear variables
time=3;%time given by user
len=0:0.1:time;
% developing 2 link robotic arm
import ETS2.*
a1 = 1; a2 =1;
%DH parameters
E = Rz('q1') * Tx(a1) * Rz('q2') * Tx(a2);
 %forward Kinematic
T0=E.fkine( [0, 0], 'deg');
Tf=E.fkine( [-47, 125], 'deg');
q0=T0.t;%start point of the end effector in Cartesian space
qf=Tf.t;%end point of the end effector in Cartesian space
% space trajectory formed by LSPB
[s, sd, sdd]=mtraj(@lspb, q0(1), qf(1), len);
figure(1)
mtraj(@lspb,  q0(1), qf(1), len);
[s2, sd2, sdd2]=mtraj(@lspb, q0(2), qf(2), len);
figure(2)
mtraj(@lspb,q0(2), qf(2), len);
%calculating the joint angles by inverse kinematic
q=zeros(length(len),2);
xv=s;
yv=s2;
mdl_planar2
Tm=zeros(4,4,length(len));
for i=1:length(len)
    Tm(:,:,i)=transl(xv(i),yv(i),0);
    qq=p2.ikine( Tm(:,:,i),[0 0],'mask',[1 1 0 0 0 0]);
    if(isempty(qq))
        qq=[0 0];
    end
    q(i,1)=qq(1);
    q(i,2)=qq(2);
end
%plot 
figure(3)
plot(s,s2);
hold on
grid on
scatter(q0(1),q0(2))
scatter(qf(1),qf(2),'d')
legend('locus','start point','end point')
title('The locus of the end effector in Cartesian space')
xlabel('joint1')
ylabel('joint2')
hold off

figure(4)
hold on
grid on
plot(len',sd,len',sd2);
title('Velocity over time')
xlabel('time[s]')
ylabel('velocity[m/s]')
legend('joint 1','joint 2')
hold off

figure(5)
hold on
grid on
plot(len',sdd,len',sdd2);
title('Acceleration over time')
xlabel('time[s]')
ylabel('Acceleration[m/s^2]')
legend('joint 1','joint 2')
hold off

figure(6)
hold on
grid on
plot(len',q(:,1),len',q(:,2));
title('Joint angles over time')
xlabel('time[s]')
ylabel('Angle in Radian')
legend('joint 1','joint 2')
hold off