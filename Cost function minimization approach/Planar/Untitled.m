clc
close all
clear all

C=[1, 2, 1];
P=[4, 4, 1];
L=[-1, 1, 0];
ang=pi/8;

T1=[1,0,-C(1);0,1,-C(2);0,0,1];
R=[cos(ang),-sin(ang),0;sin(ang),cos(ang),0;0,0,1];
T2=[1,0,C(1);0,1,C(2);0,0,1];

nP=T2*R*T1*P';

hold on
plot(C(1),C(2),'*k')
plot(P(1),P(2),'*b')
plot(nP(1),nP(2),'*g')
axis([0,10,0,10])

nL=L*inv(T2*R*T1);
fplot(@(x) (-L(3)-L(1)*x)/L(2))
fplot(@(x) (-nL(3)-nL(1)*x)/nL(2))