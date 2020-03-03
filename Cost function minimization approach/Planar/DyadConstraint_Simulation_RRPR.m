clc
clear all
close all

Simulate()

function []= Simulate ()
Points= [[0, 1]; [.81, 1.59]; [-1.14, -1.39]];
Lines= [-1.2609, 1, -5.2284];
Links= {{'P1','P2','I'},{'P2','L1'},{'L1','P3'},{'P3','P1','G'}};

% Fixed- P1,L1, Input- P2, State- P3
% Starting guess
x0 = [Lines(1),Lines(3)];

for incr=pi/10:pi/90:2*pi
    InpGrndPivot=Points(1,:);
    InpMovPivot=Points(2,:);
    InpLen=pdist2(InpGrndPivot,InpMovPivot);
    InpAng=atan2(InpMovPivot(2)-InpGrndPivot(2), InpMovPivot(1)-InpGrndPivot(1));
    Inpx=InpGrndPivot(1)+InpLen*cos(InpAng+incr);
    Inpy=InpGrndPivot(2)+InpLen*sin(InpAng+incr);
    Inp=[Inpx, Inpy];
    l2=perpDistLinePt(Lines,Points(2,:));
    l3=perpDistLinePt(Lines,Points(3,:));
    
    Cost=@(x) constraint(l2, l3, Inp, x(1), x(2), Points(3,:));
    options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    [x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    
    L=[x(1),1,x(2)];
    figure(1)
    %Link1
    plot([Points(1,1),Inpx],[Points(1,2),Inpy],'LineWidth',5)
    hold on
    %Link2
    Pt1= nearestPt (L, Points(2,:));
    plot([Inpx,Pt1(1)],[Inpy,Pt1(2)],'LineWidth',5)
    %Link3
    %Find perp point on L1 from P3
    Pt2= nearestPt (L, Points(3,:));
    plot([Pt2(1),Points(3,1)],[Pt2(2),Points(3,2)],'LineWidth',5)
    %Line
    a=L(1); b=L(2); c=L(3);
    Lin=@ (x) -(a*x+c)/b;
    plot([-6,2], [Lin(-6),Lin(2)],'-.')
    xlim([-6 2]) 
    ylim([-4 4])
    hold off
    drawnow;
end
end

% constraints
function [cost]= constraint (l2, l3, P2, L1, L3, P3)
cost=0;
L2=1;
% RP1
c1=(P2(1)*L1+P2(2)*L2+L3)^2-l2^2*(L1^2+L2^2);
% RP2
c2=(P3(1)*L1+P3(2)*L2+L3)^2-l3^2*(L1^2+L2^2);

cost=c1^2+c2^2;
end

function [r]= perpDistLinePt (L, P)
r=abs(L(1)*P(1)+L(2)*P(2)+L(3))/sqrt(L(1)^2+L(2)^2);
end

function [Pt]= nearestPt (L, P)
a=L(1); b=L(2); c=L(3);
Px=P(1); Py=P(2);
x=(b*(b*Px-a*Py)-a*c)/(a^2+b^2);
y=(a*(-b*Px+a*Py)-b*c)/(a^2+b^2);
Pt=[x,y];
end


