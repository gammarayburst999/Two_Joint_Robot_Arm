clear variables
close all;
time=3;%time given by user
di=0.01;
len=0:di:time;
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
mdl_planar2
Ts=transl(q0(1),q0(2),0);
Te=transl(qf(1),qf(2),0);
% space trajectory formed by CTRAJ
Tfinal=ctraj(Ts, Te, length(len));
%calculating the joint angles by inverse kinematic
mdl_planar2
q=zeros(length(len),2);
qd=zeros(length(len),2);
qdd=zeros(length(len),2);
xd=zeros(length(len),2);
xdd=zeros(length(len),2);
xv=zeros(length(len),1);
yv=zeros(length(len),1);
for i=1:length(len)
    qq3=p2.ikine( Tfinal(:,:,i),[0 0],'mask',[1 1 0 0 0 0]);
    if(isempty(qq3))
        qq3=[0 0];
    end
    q(i,1)=qq3(1);
    q(i,2)=qq3(2);
    aa= Tfinal(:,:,i);
    xv(i)=aa(1,4);
    yv(i)=aa(2,4);
end
% calculating angular velocity
for i2=1:length(len)
    qd(1,1)=0;
    qd(1,2)= 0;
    if(i2>1)
        qd(i2,1)=(q(i2,1)-q(i2-1,1))/di;
        qd(i2,2)=(q(i2,2)-q(i2-1,2))/di;
    end
end
% calculating angular acceleration
for i3=1:length(len)
    qdd(1,1)=0;
    qdd(1,2)= 0;
    if(i3>1)
        qdd(i3,1)=(qd(i3,1)-qd(i3-1,1))/di;
        qdd(i3,2)=(qd(i3,2)-qd(i3-1,2))/di;
    end
end
% calculating  velocity
for i4=1:length(len)
    J11=(-a1*sin(q(i4,1))-a2*sin(q(i4,1)+q(i4,2)));
    J12=(-a2*sin(q(i4,1)+q(i4,2)));
    J21=(a1*cos(q(i4,1))+a2*cos(q(i4,1)+q(i4,2)));
    J22=(a2*cos(q(i4,1)+q(i4,2)));
    xd(i4,1)=J11*qd(i4,1)+J12*qd(i4,2);
    xd(i4,2)=J21*qd(i4,1)+J22*qd(i4,2);
end
% calculating  accelation
for i5=1:length(len)
    J11=(-a1*sin(q(i5,1))-a2*sin(q(i5,1)+q(i5,2)));
    J12=(-a2*sin(q(i5,1)+q(i5,2)));
    J21=(a1*cos(q(i5,1))+a2*cos(q(i5,1)+q(i5,2)));
    J22=(a2*cos(q(i5,1)+q(i5,2)));
    M11=((-a1*cos(q(i5,1))*qd(i5,1)-a2*cos(q(i5,1)+q(i5,2))*...
        (qd(i5,1)+qd(i5,2)))*qd(i5,1));
    M12=-a2*cos(q(i5,1)+q(i5,2))*(qd(i5,1)+qd(i5,2))*qd(i5,2);
    M21=((-a1*sin(q(i5,1))*qd(i5,1)-a2*sin(q(i5,1)+q(i5,2))*...
        (qd(i5,1)+qd(i5,2)))*qd(i5,1));
    M22=-a2*sin(q(i5,1)+q(i5,2))*(qd(i5,1)+qd(i5,2))*qd(i5,2);
    xdd(i5,1)=J11*qdd(i5,1)+J12*qdd(i5,2)+M11+M12;
    xdd(i5,2)=J21*qdd(i5,1)+J22*qdd(i5,2)+M21+M22;
end
%Plot
figure(1)
plot(xv(:,1),yv(:,1));
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
plot(len',q(:,1),len',q(:,2));
title('Joint angles over time')
xlabel('time[s]')
ylabel('Angle in Radian')
legend('joint 1','joint 2')
hold off

figure(3)
hold on
grid on
plot(len',xd(:,1),len',xd(:,2));
title('Velocity over time')
xlabel('time[s]')
ylabel('velocity[m/s]')
legend('joint 1','joint 2')
hold off

figure(4)
hold on
grid on
plot(len',xdd(:,1),len',xdd(:,2));
title('Acceleration over time')
xlabel('time[s]')
ylabel('Acceleration[m/s^2]')
legend('joint 1','joint 2')
hold off

