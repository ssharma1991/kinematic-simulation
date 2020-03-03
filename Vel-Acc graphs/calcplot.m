clc
close all

curve=Cplr; %n pts- 2D or 3D
phi=inp_ang; %n input angles
omega=2*pi; %rad/sec
Delta_phi=phi(2)-phi(1);
Delta_time=Delta_phi/omega; 
t=phi/omega;

Vel=(curve(2:end,:)-curve(1:end-1,:))/Delta_time;
Speed=sqrt(sum(Vel.^2,2));

fig=figure(1);
set(fig, 'Position', [100 100 1200 700])

subplot(2,1,1)
grid on
%plot(inp(2:end),Speed)
hold on
plot(t(1:end-1),Vel(:,1),'LineWidth',5);
plot(t(1:end-1),Vel(:,2),'LineWidth',5);
plot(t(1:end-1),Vel(:,3),'LineWidth',5);
hold off
axis ([-.15,.3,-inf,inf])
lgd=legend('x component','y component','z component');
lgd.FontSize=15;
xlabel('Time (s)','FontSize',15)
ylabel('Velocity (units/s)','FontSize',15)
title('Coupler curve Velocity','FontSize',15)

subplot(2,1,2)
grid on
Acc=(Vel(2:end,:)-Vel(1:end-1,:))/Delta_time;
Mag_acc=sqrt(sum(Acc.^2,2));
%plot(inp(3:end),Mag_acc)
hold on
plot(t(1:end-2),Acc(:,1),'LineWidth',5);
plot(t(1:end-2),Acc(:,2),'LineWidth',5);
plot(t(1:end-2),Acc(:,3),'LineWidth',5);
hold off
axis ([-.15,.3,-inf,inf])
lgd=legend('x component','y component','z component');
lgd.FontSize=15;
xlabel('Time (s)','FontSize',15)
ylabel('Accleration (units/s^2)','FontSize',15)
title('Coupler curve Accleration','FontSize',15)