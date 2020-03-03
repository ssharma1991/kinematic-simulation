clc
clear all
close all

Pts=[-0.825,-1.033;-1.249,-0.600;-2.918,0.360;-3.287,0.363;-3.521,0.302;-4.038,-0.178;-4.151,-0.635;-3.541,-1.905;-1.864,-2.352;-0.692,-1.442;];
MechParam=[3.956, 1.319, 3.440, 5.755, -2.412, -0.322, -0.316, 0.809, 3.115, 0.280];
TaskFDs=[-0.002 - 0.000i,0.004 - 0.018i,0.057 - 0.037i,0.132 - 0.180i,-2.496 - 1.052i,1.484 + 0.127i,-0.025 + 0.027i,0.012 + 0.090i,0.009 + 0.009i];
PhiMax=5.9751;
CouplerFDs=[0.008 - 0.007i,0.021 - 0.016i,0.055 - 0.046i,0.132 - 0.177i,-2.496 - 1.052i,1.484 + 0.127i,-0.009 + 0.039i,-0.005 + 0.009i,-0.003 + 0.002i];

hold on
axis equal off
drawTaskCurve(TaskFDs, PhiMax)
drawPoints(Pts)
[F1,F2,M1,M2,C]=toPtParam(MechParam,Pts(1,:));
drawCouplerPath(calcCouplerPath(F1,F2,M1,M2,C));
draw4bar(F1,F2,M1,M2,C);
%legend('Path points','Task curve','Coupler curve');
%simulate4bar(F1,F2,M1,M2,C);


%Simulate four-bar
function [] = simulate4bar(F1,F2,M1,M2,C)
path=calcCouplerPath(F1,F2,M1,M2,C);
l1=pdist([F1;F2]);
l2=pdist([F1;M1]);
l3=pdist([M1;M2]);
l4=pdist([F2;M2]);
theta1=atan2(F2(2)-F1(2),F2(1)-F1(1));
r=pdist([M1;C]);
alpha=atan2(C(2)-M1(2),C(1)-M1(1))-atan2(M2(2)-M1(2),M2(1)-M1(1));
M2_t=M2;
phi_0=atan2(M1(2)-F1(2),M1(1)-F1(1))-theta1;

for phi=phi_0:2*pi/360:2*pi+phi_0
    phi_t=theta1+phi;
    if checkfeasibility(l1,l2,l3,l4,phi_t)
        M1_t_cmplx=complex(F1(1),F1(2))+l2*exp(1i*phi_t);
        M1_t=[real(M1_t_cmplx) imag(M1_t_cmplx)];
        
        % Circle circle intersection
        A = M1_t; %# center of the first circle
        B = F2; %# center of the second circle
        r2 = l4; %# radius of the SECOND circle
        r1 = l3; %# radius of the FIRST circle
        dist = norm(A-B); %# distance between circles
        cosAlpha = (r1^2+dist^2-r2^2)/(2*r1*dist);
        u_AB = (B - A)/dist; %# unit vector from first to second center
        pu_AB = [u_AB(2), -u_AB(1)]; %# perpendicular vector to unit vector
        intersect_1 = A + u_AB * (r1*cosAlpha) + pu_AB * (r1*sqrt(1-cosAlpha^2));
        intersect_2 = A + u_AB * (r1*cosAlpha) - pu_AB * (r1*sqrt(1-cosAlpha^2));
        if pdist([M2_t;intersect_1])<pdist([M2_t;intersect_2])
            M2_t=intersect_1;
        else
            M2_t=intersect_2;
        end
        
        lamda_t=atan2(M2_t(2)-M1_t(2),M2_t(1)-M1_t(1));
        C_t_cmplx=M1_t_cmplx+r*exp(1i*(lamda_t+alpha));
        C_t=[real(C_t_cmplx) imag(C_t_cmplx)];
        
        cla
        hold on
        axis equal
        draw4bar(F1,F2,M1_t,M2_t,C_t)
        drawCouplerPath(path)
        drawnow
    else
        break;
    end
end
end
function [feasibility] = checkfeasibility(l1,l2,l3,l4,phi_t)
l21=l2/l1;
l31=l3/l1;
l41=l4/l1;
feasibility=((l31-l41)^2<=(1+l21^2-2*l21*cos(phi_t)) && (1+l21^2-2*l21*cos(phi_t))<=(l31+l41)^2);
end

%Draw four-bar mechanism
function [] =draw4bar(F1,F2,M1,M2,C)
offset=.1;
drawFP(F1,offset);
drawFP(F2,offset);
drawLink(F1,M1,offset);
drawLink(F2,M2,offset);
drawCoupler(M1, M2, C, offset);
drawRev([F1;F2;M1;M2;C]);
end
%Draw link
function [] = drawLink(P1, P2, offset)
[Pt1,Pt2]=findOffset(P1,P2,offset);
[Pt3,Pt4]=findOffset(P1,P2,-offset);
path=lineseg(Pt1,Pt2);
path=cat(1,path,arc(Pt2,Pt4,P2));
path=cat(1,path,lineseg(Pt3,Pt4));
path=cat(1,path,arc(Pt3,Pt1,P1));
%plot(path(:,1),path(:,2),'k');
p=fill(path(:,1),path(:,2),[.8,1,.5]);
set(p,'facealpha',.7);
end
function [O1,O2] = findOffset(P1,P2,offset)
dirn=(P2-P1)/pdist([P1;P2]);
perpDirn=[dirn(2) -dirn(1)];
O1=P1+offset*perpDirn;
O2=P2+offset*perpDirn;
end
function [path] = arc(P1, P2, C)
d1=P1-C;
t1=atan2(d1(2),d1(1));
d2=P2-C;
t2=atan2(d2(2),d2(1));
t2=mod(t2-t1,2*pi)+t1; %to ensure arcs from t1 to t2 counter-clockwise
t = linspace(t1,t2,20);
R=pdist([P1;C]);
x = C(1)+R*cos(t);
y = C(2)+R*sin(t);
path=[x',y'];
end
function [path] = lineseg(P1, P2)
x=[P1(1), P2(1)];
y=[P1(2), P2(2)];
path=[x',y'];
end
%Draw coupler
function [] = drawCoupler(P1, P2, P3, offset)
[Pt1,Pt2]=findOffset(P1,P2,offset);
[Pt3,Pt4]=findOffset(P2,P3,offset);
[Pt5,Pt6]=findOffset(P3,P1,offset);
path=lineseg(Pt1,Pt2);
path=cat(1,path,arc(Pt2,Pt3,P2));
path=cat(1,path,lineseg(Pt3,Pt4));
path=cat(1,path,arc(Pt4,Pt5,P3));
path=cat(1,path,lineseg(Pt5,Pt6));
path=cat(1,path,arc(Pt6,Pt1,P1));
%plot(path(:,1),path(:,2),'k');
p=fill(path(:,1),path(:,2),[1,.7,.85]);
set(p,'facealpha',.7);
end
%Draw fixed pivot
function [] = drawFP(Pt,offset)
L=2*offset;
x=[Pt(1), Pt(1)-L/2, Pt(1)+L/2, Pt(1)];
y=[Pt(2), Pt(2)-L, Pt(2)-L, Pt(2)];
%plot(x,y,'k','LineWidth',2)
fill(x,y,[.85,.85,.85],'LineWidth',2);
end
%Draw revolute joints
function [] = drawRev(Pts)
x=Pts(:,1);
y=Pts(:,2);
plot(x,y,'k.','MarkerSize',15);
end
%Draw coupler path
function [] = drawCouplerPath(path)
plot(path(:,1),path(:,2),'-','Color',[.16,.45,.75,.7],'LineWidth',3);
end
function [path] = calcCouplerPath(F1,F2,M1,M2,C)
path=[];
l1=pdist([F1;F2]);
l2=pdist([F1;M1]);
l3=pdist([M1;M2]);
l4=pdist([F2;M2]);
theta1=atan2(F2(2)-F1(2),F2(1)-F1(1));
r=pdist([M1;C]);
alpha=atan2(C(2)-M1(2),C(1)-M1(1))-atan2(M2(2)-M1(2),M2(1)-M1(1));
M2_t=M2;
phi_0=atan2(M1(2)-F1(2),M1(1)-F1(1))-theta1;

for phi=phi_0:2*pi/360:2*pi+phi_0
    phi_t=theta1+phi;
    if checkfeasibility(l1,l2,l3,l4,phi_t)
        M1_t_cmplx=complex(F1(1),F1(2))+l2*exp(1i*phi_t);
        M1_t=[real(M1_t_cmplx) imag(M1_t_cmplx)];
        
        % Circle circle intersection
        A = M1_t; %# center of the first circle
        B = F2; %# center of the second circle
        r2 = l4; %# radius of the SECOND circle
        r1 = l3; %# radius of the FIRST circle
        dist = norm(A-B); %# distance between circles
        cosAlpha = (r1^2+dist^2-r2^2)/(2*r1*dist);
        u_AB = (B - A)/dist; %# unit vector from first to second center
        pu_AB = [u_AB(2), -u_AB(1)]; %# perpendicular vector to unit vector
        intersect_1 = A + u_AB * (r1*cosAlpha) + pu_AB * (r1*sqrt(1-cosAlpha^2));
        intersect_2 = A + u_AB * (r1*cosAlpha) - pu_AB * (r1*sqrt(1-cosAlpha^2));
        if pdist([M2_t;intersect_1])<pdist([M2_t;intersect_2])
            M2_t=intersect_1;
        else
            M2_t=intersect_2;
        end
        
        lamda_t=atan2(M2_t(2)-M1_t(2),M2_t(1)-M1_t(1));
        C_t_cmplx=M1_t_cmplx+r*exp(1i*(lamda_t+alpha));
        C_t=[real(C_t_cmplx) imag(C_t_cmplx)];
        
        path=cat(1,path,C_t);
    else
        break;
    end
end
end

%Convert to FD-approach design param
function [param] = toFDParam(F1,F2,M1,M2,C)
l1=pdist([F1;F2]);
l2=pdist([F1;M1]);
l3=pdist([M1;M2]);
l4=pdist([F2;M2]);
r=pdist([M1;C]);
x0=F1(1);
y0=F1(2);
theta1=atan2(F2(2)-F1(2),F2(1)-F1(1));
alpha=atan2(C(2)-M1(2),C(1)-M1(1))-atan2(M2(2)-M1(2),M2(1)-M1(1));
phi0=0;
param=[l1 l2 l3 l4 r x0 y0 theta1 alpha phi0];
end
function [F1,F2,M1,M2,C]= toPtParam(param, Pt)
l1=param(1);
l2=param(2);
l3=param(3);
l4=param(4);
r=param(8);
x0=param(5);
y0=param(6);
theta1=param(7);
alpha=param(9);
phi0=param(10);
F1=[x0,y0];
F2=F1+[real(l1*exp(1i*theta1)) imag(l1*exp(1i*theta1))];
M1=F1+[real(l2*exp(1i*(theta1+phi0))) imag(l2*exp(1i*(theta1+phi0)))];

% Circle circle intersection
A = M1; %# center of the first circle
B = F2; %# center of the second circle
r2 = l4; %# radius of the SECOND circle
r1 = l3; %# radius of the FIRST circle
dist = norm(A-B); %# distance between circles
cosAlpha = (r1^2+dist^2-r2^2)/(2*r1*dist);
u_AB = (B - A)/dist; %# unit vector from first to second center
pu_AB = [u_AB(2), -u_AB(1)]; %# perpendicular vector to unit vector
intersect_1 = A + u_AB * (r1*cosAlpha) + pu_AB * (r1*sqrt(1-cosAlpha^2));
intersect_2 = A + u_AB * (r1*cosAlpha) - pu_AB * (r1*sqrt(1-cosAlpha^2));
lambdaAndTheta1_1=atan2(intersect_1(2)-M1(2),intersect_1(1)-M1(1));
C_1=M1+[real(r*exp(1i*(lambdaAndTheta1_1+alpha))) imag(r*exp(1i*(lambdaAndTheta1_1+alpha)))];
lambdaAndTheta1_2=atan2(intersect_2(2)-M1(2),intersect_2(1)-M1(1));
C_2=M1+[real(r*exp(1i*(lambdaAndTheta1_2+alpha))) imag(r*exp(1i*(lambdaAndTheta1_2+alpha)))];

if pdist([Pt;C_1])<pdist([Pt;C_2])
    M2=intersect_1;
    C=C_1;
else
    M2=intersect_2;
    C=C_2;
end

end

%Draw Task curve
function [] = drawTaskCurve(FDs, phiMax)
sample_pts=360;
har=(length(FDs)-1)/2;
path=[];
for i=1:sample_pts
    theta=(i-1)*(phiMax)/(sample_pts-1);
    z=0;
    for n=-har:har
        z=z+FDs(n+har+1)*exp(n*1i*theta);
    end
    path=cat(1,path,z);
end
plot(real(path),imag(path),':','Color',[.2,.6,.2],'LineWidth',3);
end
%Draw Points
function [] = drawPoints(Pts)
plot(Pts(:,1),Pts(:,2),'k+','MarkerSize',30,'LineWidth',1);
plot(Pts(:,1),Pts(:,2),'ko','MarkerSize',15,'LineWidth',1);
end

