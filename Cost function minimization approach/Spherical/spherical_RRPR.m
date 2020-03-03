clc
clear all
close all

[Cplr,inp_ang]=Simulate()

function [CplrPath,ang]= Simulate ()
Points= [[.8, .2, .2]; [.9, .3, .6]; [-.5, 1, .7]; [.6, -.25, 1]];
Plane=[.8, -.8, .3];
Links= {{'P1','P2','I'},{'P2','Pl1'},{'Pl1','P3'},{'P3','P1','G'}};

Points(1,:)=Points(1,:)/norm(Points(1,:));
Points(2,:)=Points(2,:)/norm(Points(2,:));
Points(3,:)=Points(3,:)/norm(Points(3,:));
Points(4,:)=Points(4,:)/norm(Points(4,:));
Plane=Plane/norm(Plane);

% Fixed- P1,P3, Input- P2, State- Pl1
% Starting guessfliplr(CplrPath)
x0 = cat(2,Plane,Points(4,:));
ang1=RPconstAlt (Points(2,:),Plane);
ang2=RPconstAlt (Points(3,:),Plane);
angC1=RRconstAlt (Points(4,:),Points(2,:));
angC2=RPconstAlt (Points(4,:),Plane);

CplrPath=[];
ang=[];
for incr=0:pi/90:2*pi
    InpPt=Points(1,:);
    MovingPt=Points(2,:);
    a=InpPt(1);b=InpPt(2);c=InpPt(3);
    d=sqrt(b^2+c^2);
    Rotx=[1,0,0;0,c/d,-b/d;0,b/d,c/d];
    Roty=[d,0,-a;0,1,0;a,0,d];
    Rotz=[cos(incr),-sin(incr),0;sin(incr),cos(incr),0;0,0,1];
    newMovingPt=inv(Rotx)*inv(Roty)*Rotz*Roty*Rotx*MovingPt';
    
    Cost=@(x) constraintAlt(ang1,ang2,newMovingPt',x,Points(3,:),angC1,angC2);
    options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    [x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    if (abs(fval)>10^-5)
        break
    end
    figure(1)
    cla
    DrawSphere([0,0,0],1);
    hold on
    DrawSpLink(Points(1,:),newMovingPt');
    n_pt1=PlanePt2NearestPt(newMovingPt',x(1:3));
    DrawSpLink(newMovingPt',n_pt1);
    n_pt2=PlanePt2NearestPt(Points(3,:),x(1:3));
    DrawSpLink(n_pt2,Points(3,:));
    DrawSpLink(newMovingPt',x(4:6));
    DrawSpLink(n_pt1,x(4:6));
    plotCircle3D([0,0,0],x(1:3),1);
    CplrPath=cat(1,CplrPath,x(4:6));
    ang=cat(1,ang,incr);
end

plot3(CplrPath(:,1),CplrPath(:,2),CplrPath(:,3))

end

%CONSTRAINTS FUNCTIONS
function [cost]= constraint (ang1,ang2,P2,x,P3,angC1,angC2)
Pl=x(1:3);
C=x(4:6);
c1=ang1-RPconst (P2,Pl);
c2=ang2-RPconst (P3,Pl);
c3=angC1-RRconst (C,P2);
c4=angC2-RPconst (C,Pl);
cost=c1^2+c2^2+c3^2+c4^2+(1-norm(C))^2;
end
function [cost]= RPconst (Pt,Pl)
% calc nearest point on plane
N_pt=PlanePt2NearestPt(Pt,Pl);
cost=dot(N_pt,Pt)/(norm(N_pt)*norm(Pt)); % PROBLEM- norm(N_pt)=0
end
function [N_pt]=PlanePt2NearestPt(Pt,Pl)
t=(Pt(1)*Pl(1)+Pt(2)*Pl(2)+Pt(3)*Pl(3))/norm(Pl)^2;
N_pt=Pt-t*Pl;
end
%Alternate CONSTRAINTS FUNCTIONS
function [cost]= constraintAlt (ang1,ang2,P2,x,P3,angC1,angC2)
Pl=x(1:3);
C=x(4:6);
c1=ang1-RPconstAlt (P2,Pl);
c2=ang2-RPconstAlt (P3,Pl);
c3=angC1-RRconstAlt (C,P2);
c4=angC2-RPconstAlt (C,Pl);
cost=c1^2+c2^2+c3^2+c4^2+(dot(C,C)-1)^2;
end
function [cost]= RPconstAlt (Pt,Pl)
cost=dot(Pl,Pt)/norm(Pl);
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
function plotCircle3D(center,normal,radius)
% center is an array [x,y,z]
% normal is an array [nx,ny,nz]
% radius is a value
% E.g., plotCircle3D([0,0,0],[-1,0,0],1)
theta=0:0.01:2*pi;
v=null(normal);
points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
plot3(points(1,:),points(2,:),points(3,:),'r:','linewidth',2);
end