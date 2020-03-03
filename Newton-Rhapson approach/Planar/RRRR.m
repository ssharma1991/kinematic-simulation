clc
clear all
close all

Simulate()

function []= Simulate ()
Points= [[0, 0]; [0, 1]; [1, 2]; [2, 0]];
Links= {{'P1','P2','I'},{'P2','P3'},{'P3','P4'},{'P4','P1','G'}};

% Fixed- P1,P4, Input- P2, State- P3
P1=Points(1,:);
P2=Points(2,:);
P3=Points(3,:);
P4=Points(4,:);

% Starting guess
x0 = Points(3,:);
l2=RRconstAlt(P2,P3);
l3=RRconstAlt(P3,P4);

tic
for incr=0:pi/1800:2*pi
    InpPt=P1;
    MovingPt=[P2,1];
    
    T1=[1,0,-InpPt(1);0,1,-InpPt(2);0,0,1];
    R=[cos(incr),-sin(incr),0;sin(incr),cos(incr),0;0,0,1];
    T2=[1,0,InpPt(1);0,1,InpPt(2);0,0,1];
    Trans=T2*R*T1;
    newMovingPt=Trans*MovingPt';
    P2new=newMovingPt(1:2)';
    
    x=NewtonRhapson(l2,l3,P2new,x0,P4);
    %Cost=@(x) constraintAlt(ang1,ang2,newMovingPt,x,Points(4,1:2));
    %options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    %[x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    
%     figure(1)
%     off=.1;
%     cla
%     drawLink(InpPt,newMovingPt(1:2)',off);
%     hold on
%     drawnow
%     drawLink(x0,newMovingPt(1:2)',off);
%     drawLink(x0,Points(4,:),off);
%     drawRev(x0);
%     drawRev(newMovingPt(1:2)');
%     drawRev(InpPt);
%     drawRev(Points(4,:));
%     drawFP(InpPt,off);
%     drawFP(Points(4,:),off);
%     axis ([-1.5,2.5,-1.5,2.5])
%     axis square
%     if incr==0
%         gif('plRRRR.gif');
%     else
%         gif
%     end
end
toc
end


function [cost]= RRconstAlt (P1,P2)
cost=norm(P1-P2);
end

function [x]= NewtonRhapson(l2,l3,P2,P3,P4)
x=P3;
err=norm(res(l2,l3,P2,x,P4));
while (err>10^-8)
    J=Jac(l2,l3,P2,x,P4);
    R=res(l2,l3,P2,x,P4);
    x=x'-J\R;
    x=x';
    err=norm(res(l2,l3,P2,x,P4));
end
end
function [residual]= res (l2,l3,P2,P3,P4)
c1=(l2^2-RRconstAlt(P3,P2)^2);
c2=(l3^2-RRconstAlt(P3,P4)^2);
residual=[c1;c2];
end
function [J]= Jac(l2,l3,P2,P3,P4)
Dc1_x=2*(-P3(1)+P2(1));
Dc1_y=2*(-P3(2)+P2(2));
Dc2_x=2*(-P3(1)+P4(1));
Dc2_y=2*(-P3(2)+P4(2));
J=[Dc1_x,Dc1_y;Dc2_x,Dc2_y];
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

%Draw fixed pivot
function [] = drawFP(Pt,offset)
L=2*offset;
x=[Pt(1), Pt(1)-L/2, Pt(1)+L/2, Pt(1)];
y=[Pt(2), Pt(2)-L, Pt(2)-L, Pt(2)];
%plot(x,y,'k','LineWidth',2)
fill(x,y,[.85,.85,.85],'LineWidth',2);
end
function [] = drawP(Pt,offset)
L=2*offset;
x=[Pt(1)-L/2, Pt(1)+L/2, Pt(1)+L/2, Pt(1)-L/2, Pt(1)-L/2,];
y=[Pt(2)-L, Pt(2)-L, Pt(2)+L, Pt(2)+L, Pt(2)-L,];
%plot(x,y,'k','LineWidth',2)
fill(x,y,[.85,.85,.85],'LineWidth',2);
end
%Draw revolute joints
function [] = drawRev(Pts)
x=Pts(:,1);
y=Pts(:,2);
plot(x,y,'k.','MarkerSize',15);
end




