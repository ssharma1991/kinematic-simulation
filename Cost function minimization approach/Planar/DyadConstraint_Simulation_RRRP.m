clc
clear all
close all

Simulate()

function []= Simulate ()
Points= [[0, 1]; [.81, 1.59]; [1.61, -1.3]];
Lines= [.4991, 1, .4966];
Links= {{'P1','P2','I'},{'P2','P3'},{'P3','L1'},{'L1','P1','G'}};

% Fixed- P1,L1, Input- P2, State- P3
% Starting guess
x0 = Points(3,:);

for incr=pi/10:pi/90:2*pi
    InpGrndPivot=Points(1,:);
    InpMovPivot=Points(2,:);
    InpLen=pdist2(InpGrndPivot,InpMovPivot);
    InpAng=atan2(InpMovPivot(2)-InpGrndPivot(2), InpMovPivot(1)-InpGrndPivot(1));
    Inpx=InpGrndPivot(1)+InpLen*cos(InpAng+incr);
    Inpy=InpGrndPivot(2)+InpLen*sin(InpAng+incr);
    Inp=[Inpx, Inpy];
    l2=pdist2(Points(2,:),Points(3,:));
    l3=perpDistLinePt(Lines,Points(3,:));
    
    Cost=@(x) constraint(l2, l3, Inp, x(1), x(2), Lines);
    options = optimoptions(@fminunc,'Algorithm','quasi-newton');
    [x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    
    %%%%%%%%%%%%%Plot Constraint Surface
    cost=[];
    for i=-2:.1:3
        cst=[];
        for j=-2:.1:3
            cst=cat(2,cst,Cost([i,j]));
        end
        cost=cat(1,cost,cst);
    end
    i=-2:.1:3;
    j=-2:.1:3;
    surf(i,j,cost','FaceAlpha',0.5)
    view(2)
    min(min(cost))
    hold on
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
    
    figure(1)
    %Link1
    plot([Points(1,1),Inpx],[Points(1,2),Inpy],'LineWidth',5)
    hold on
    %Link2
    plot([Inpx,x(1)],[Inpy,x(2)],'LineWidth',5)
    %Link3
    %Find perp point on L1 from P3
    a=Lines(1); b=Lines(2); c=Lines(3);
    Px=(b*(b*x(1)-a*x(2))-a*c)/(a^2+b^2);
    Py=(a*(-b*x(1)+a*x(2))-b*c)/(a^2+b^2);
    plot([x(1),Px],[x(2),Py],'LineWidth',5)
    %Line
    Lin=@ (x) -(a*x+c)/b;
    plot([-2,3], [Lin(-2),Lin(3)],'-.')
    xlim([-2 3]) 
    ylim([-2 3])
    hold off
    drawnow;
end
end

% constraints
function [cost]= constraint (l2, l3, P2, P3x, P3y, L)
cost=0;
% RR
k=l2^2-P2(1)^2-P2(2)^2;
c1=2*P2(1)*P3x+2*P2(2)*P3y+k-P3x^2-P3y^2;

% RP
%c2_1=P3x*L(1)+P3y*L(2)+L(3)+l3*sqrt(L(1)^2+L(2)^2);
%c2_2=P3x*L(1)+P3y*L(2)+L(3)-l3*sqrt(L(1)^2+L(2)^2);
%c2_1=(P3x*L(1)+P3y*L(2)+L(3))/sqrt(L(1)^2+L(2)^2)-l3;
%c2_2=(P3x*L(1)+P3y*L(2)+L(3))/sqrt(L(1)^2+L(2)^2)+l3;
%c2=min(abs(c2_1),abs(c2_2));
c2=(P3x*L(1)+P3y*L(2)+L(3))^2-l3^2*(L(1)^2+L(2)^2);

cost=c1^2+c2^2;
end

function [r]= perpDistLinePt (L, P)
r=abs(L(1)*P(1)+L(2)*P(2)+L(3))/sqrt(L(1)^2+L(2)^2);
end
