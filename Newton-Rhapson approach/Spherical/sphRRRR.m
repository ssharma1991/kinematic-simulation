clc
clear all
close all

Simulate()

function []= Simulate ()
Points= [[1,0,0]; [0.8668,0.4986,0.0045]; [0.4292,0.2571,0.8659];[0,1,0]];
Links= {{'P1','P2','I'},{'P2','P3'},{'P3','P4'},{'P4','P1','G'}};

% Fixed- P1,P4, Input- P2, State- P3
P1=Points(1,:)/norm(Points(1,:));
P2=Points(2,:)/norm(Points(2,:));
P3=Points(3,:)/norm(Points(3,:));
P4=Points(4,:)/norm(Points(4,:));

% Starting guess
x0 = P3;
l2=RRconstAlt(P2,P3);
l3=RRconstAlt(P3,P4);

tic
for incr=0:-pi/1800:-2*pi
    InpPt=P1;
    MovingPt=P2;
    
    a=InpPt(1);b=InpPt(2);c=InpPt(3);
    d=sqrt(b^2+c^2);
    if (d~=0)
        Rotx=[1,0,0;0,c/d,-b/d;0,b/d,c/d];
        Roty=[d,0,-a;0,1,0;a,0,d];
        Rotz=[cos(incr),-sin(incr),0;sin(incr),cos(incr),0;0,0,1];
        newMovingPt=inv(Rotx)*inv(Roty)*Rotz*Roty*Rotx*MovingPt';
    else
        Rotx=[1,0,0;0,cos(incr),sin(incr);0,-sin(incr),cos(incr)];
        newMovingPt=Rotx*MovingPt';
    end
    P2new=newMovingPt';
    
    x=NewtonRhapson(l2,l3,P2new,x0,P4);
    %Cost=@(x) constraintAlt(ang1,ang2,newMovingPt,x,Points(4,1:2));
    %options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    %[x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    
%     figure(1)
%     cla
%     DrawSphere([0,0,0],1);
%     hold on
%     plot3(0,0,0,'*');
%     DrawSpLink(P1,P2new,'b');
%     DrawSpLink(P2new,x0,'k');
%     DrawSpLink(x0,P4,'g');
%     if incr==0
%         gif('sphRRRR.gif');
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
c3=norm(P3)-1;
residual=[c1;c2;c3];
end
function [J]= Jac(l2,l3,P2,P3,P4)
Dc1_x=2*(-P3(1)+P2(1));
Dc1_y=2*(-P3(2)+P2(2));
Dc1_z=2*(-P3(3)+P2(3));
Dc2_x=2*(-P3(1)+P4(1));
Dc2_y=2*(-P3(2)+P4(2));
Dc2_z=2*(-P3(3)+P4(3));
Dc3_x=2*P3(1);
Dc3_y=2*P3(2);
Dc3_z=2*P3(3);
J=[Dc1_x,Dc1_y,Dc1_z;Dc2_x,Dc2_y,Dc2_z;Dc3_x,Dc3_y,Dc3_z];
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
view([1 1 .5]) % adjust the viewing angle
end
function []= DrawSpLink(Pt1, Pt2, color)
pt1=Pt1/norm(Pt1);
pt2=Pt2/norm(Pt2);
pts=[];
iter=50;
for i=0:iter
    pt=pt1+i*(pt2-pt1)/iter;
    pts(end+1,:)=pt/norm(pt);
end
plot3(pts(:,1),pts(:,2),pts(:,3),color,'linewidth',3);
end