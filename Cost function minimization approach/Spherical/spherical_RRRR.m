clc
clear all
close all

Simulate()

function []= Simulate ()
Points= [[0, 1, 0]; [0, 1, .5]; [.5, 0, 1]; [1, 0, 0]];
Links= {{'P1','P2','I'},{'P2','P3'},{'P3','P4'},{'P4','P1','G'}};
%Points(1,:)=Points(1,:)/norm(Points(1,:));
%Points(2,:)=Points(2,:)/norm(Points(2,:));
%Points(3,:)=Points(3,:)/norm(Points(3,:));
%Points(4,:)=Points(4,:)/norm(Points(4,:));

% Fixed- P1,P4, Input- P2, State- P3
% Starting guess
x0 = Points(3,:);
ang1=RRconstAlt(Points(2,:),Points(3,:));
ang2=RRconstAlt(Points(3,:),Points(4,:));

for incr=pi/10:pi/90:2*pi
    InpPt=Points(1,:);
    MovingPt=Points(2,:);
    a=InpPt(1);b=InpPt(2);c=InpPt(3);
    d=sqrt(b^2+c^2);
    Rotx=[1,0,0;0,c/d,-b/d;0,b/d,c/d];
    Roty=[d,0,-a;0,1,0;a,0,d];
    Rotz=[cos(incr),-sin(incr),0;sin(incr),cos(incr),0;0,0,1];
    newMovingPt=inv(Rotx)*inv(Roty)*Rotz*Roty*Rotx*MovingPt';
    
    Cost=@(x) constraintAlt(ang1,ang2,newMovingPt',x,Points(4,:));
    options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    [x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    
    figure(1)
    cla
    DrawSphere([0,0,0],1);
    hold on
    DrawSpLink(Points(1,:),newMovingPt');
    DrawSpLink(newMovingPt',x);
    DrawSpLink(x,Points(4,:));
    plot3(0,0,0,'*');
end

end

%CONSTRAINTS FUNCTIONS
function [cost]= constraint (ang1,ang2,P2,P3,P4)
c1=ang1-RRconst(P2,P3);
c2=ang2-RRconst(P3,P4);
cost=c1^2+c2^2;
end
function [cost]= RRconst (P1,P2)
cost=dot(P1,P2)/(norm(P1)*norm(P2));
end
%Alternate CONSTRAINTS FUNCTIONS
function [cost]= constraintAlt (ang1,ang2,P2,P3,P4)
c1=ang1-RRconstAlt(P2,P3);
c2=ang2-RRconstAlt(P3,P4);
cost=c1^2+c2^2;
end
function [cost]= RRconstAlt (P1,P2)
cost=norm(P1-P2);
end


%PLOTTING FUNCTIONS
function []= DrawSphere (centre,radius)
r = radius;
[x,y,z] = sphere(50);
x0 = centre(1); y0 = centre(2); z0 = centre(3);
x = x*r + x0;
y = y*r + y0;
z = z*r + z0;

lightGrey = 0.9*[1 1 1]; % It looks better if the lines are lighter
surface(x,y,z,'FaceAlpha',.8,'FaceColor', lightGrey,'EdgeColor','none')

axis equal % so the sphere isn't distorted
view([1 1 0.75]) % adjust the viewing angle
end
function []= DrawSpLink(Pt1, Pt2)
pt1=Pt1/norm(Pt1);
pt2=Pt2/norm(Pt2);
pts=[];
iter=50;
for i=0:iter
    pt=pt1+i*(pt2-pt1)/iter;
    pts(end+1,:)=pt/norm(pt);
end
plot3(pts(:,1),pts(:,2),pts(:,3),'linewidth',3);
end