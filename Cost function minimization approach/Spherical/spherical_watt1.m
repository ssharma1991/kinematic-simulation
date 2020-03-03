clc
clear all
close all

[Cplr,inp_ang]=Simulate();

function [CplrPath,ang]= Simulate ()
Points= [[0, 0, 1];
    [.5, 0, .2];
    [.5, -.1, .3];
    [.5, .5, .1];
    [.6, .4, .4];
    [.2, .1, -.1];
    [.5, -.1, .9];
    [.2, .2, .3]];
Links= {{'P1','P2','P3','I'},{'P2','P4','P5'},{'P4','P6'},{'P3','P7'},{'P5','P7','P8'},{'P1','P6','G'}};

Points(1,:)=Points(1,:)/norm(Points(1,:));
Points(2,:)=Points(2,:)/norm(Points(2,:));
Points(3,:)=Points(3,:)/norm(Points(3,:));
Points(4,:)=Points(4,:)/norm(Points(4,:));
Points(5,:)=Points(5,:)/norm(Points(5,:));
Points(6,:)=Points(6,:)/norm(Points(6,:));
Points(7,:)=Points(7,:)/norm(Points(7,:));
Points(8,:)=Points(8,:)/norm(Points(8,:));
Points
PlotData=[];

% Fixed- P1,P6, Input- P2,P3, State- P4,P5,P7,P8
% Starting guess
x0 = [Points(4,:),Points(5,:),Points(7,:),Points(8,:)];
angL2C1=RRconstAlt(Points(2,:),Points(4,:));
angL2C2=RRconstAlt(Points(2,:),Points(5,:));
angL2C3=RRconstAlt(Points(4,:),Points(5,:));
angL3C1=RRconstAlt(Points(4,:),Points(6,:));
angL4C1=RRconstAlt(Points(3,:),Points(7,:));
angL5C1=RRconstAlt(Points(5,:),Points(7,:));
angL5C2=RRconstAlt(Points(5,:),Points(8,:));
angL5C3=RRconstAlt(Points(7,:),Points(8,:));
initConst=[angL2C1, angL2C2, angL2C3, angL3C1, angL4C1, angL5C1, angL5C2, angL5C3];

CplrPath=[];
ang=[];
for incr=0:-pi/90:-2*pi
    InpPt1=Points(1,:);
    Pt2=Points(2,:);
    Pt3=Points(3,:);
    a=InpPt1(1);b=InpPt1(2);c=InpPt1(3);
    d=sqrt(b^2+c^2);
    Rotx=[1,0,0;0,c/d,-b/d;0,b/d,c/d];
    Roty=[d,0,-a;0,1,0;a,0,d];
    Rotz=[cos(incr),-sin(incr),0;sin(incr),cos(incr),0;0,0,1];
    newPt2=inv(Rotx)*inv(Roty)*Rotz*Roty*Rotx*Pt2';
    newPt3=inv(Rotx)*inv(Roty)*Rotz*Roty*Rotx*Pt3';
    knownPts=[InpPt1;newPt2';newPt3';Points(6,:)];
    
    Cost=@(x) constraintAlt(initConst, knownPts, x);
    options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    [x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    
    if (fval>10^-9)
        break
    end
    
    P1=knownPts(1,:);
    P2=knownPts(2,:);
    P3=knownPts(3,:);
    P6=knownPts(4,:);
    P4=x(1:3);
    P5=x(4:6);
    P7=x(7:9);
    P8=x(10:12);
    %plotState(P1,P2,P3,P4,P5,P6,P7,P8);
    
    PlotData=cat(1,PlotData,{[P1;P2;P3;P4;P5;P6;P7;P8]});
    CplrPath=cat(1,CplrPath,P8);
    ang=cat(1,ang,incr);
end
PlotData=flipud(PlotData);
CplrPath=flipud(CplrPath);
ang=flipud(ang);

x0 = [Points(4,:),Points(5,:),Points(7,:),Points(8,:)];
for incr=pi/90:pi/90:2*pi
    InpPt1=Points(1,:);
    Pt2=Points(2,:);
    Pt3=Points(3,:);
    a=InpPt1(1);b=InpPt1(2);c=InpPt1(3);
    d=sqrt(b^2+c^2);
    Rotx=[1,0,0;0,c/d,-b/d;0,b/d,c/d];
    Roty=[d,0,-a;0,1,0;a,0,d];
    Rotz=[cos(incr),-sin(incr),0;sin(incr),cos(incr),0;0,0,1];
    newPt2=inv(Rotx)*inv(Roty)*Rotz*Roty*Rotx*Pt2';
    newPt3=inv(Rotx)*inv(Roty)*Rotz*Roty*Rotx*Pt3';
    knownPts=[InpPt1;newPt2';newPt3';Points(6,:)];
    
    Cost=@(x) constraintAlt(initConst, knownPts, x);
    options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    [x,fval,exitflag,output] = fminunc(Cost,x0,options);
    x0=x;
    
    if (fval>10^-9)
        break
    end
    
    P1=knownPts(1,:);
    P2=knownPts(2,:);
    P3=knownPts(3,:);
    P6=knownPts(4,:);
    P4=x(1:3);
    P5=x(4:6);
    P7=x(7:9);
    P8=x(10:12);
    %plotState(P1,P2,P3,P4,P5,P6,P7,P8);
    
    PlotData=cat(1,PlotData,{[P1;P2;P3;P4;P5;P6;P7;P8]});
    CplrPath=cat(1,CplrPath,P8);
    ang=cat(1,ang,incr);
end

figure('Name','Simulation','Position',[0,0,1000,1000])
for i=1:length(CplrPath)
    P1=PlotData{i}(1,:);
    P2=PlotData{i}(2,:);
    P3=PlotData{i}(3,:);
    P4=PlotData{i}(4,:);
    P5=PlotData{i}(5,:);
    P6=PlotData{i}(6,:);
    P7=PlotData{i}(7,:);
    P8=PlotData{i}(8,:);
    plotState(P1,P2,P3,P4,P5,P6,P7,P8);
    DrawCplrPath(CplrPath);
    if i==1
        gif('watt1OpenLoop1.gif');
    else
        gif
    end
end

for i=length(CplrPath):-1:1
    P1=PlotData{i}(1,:);
    P2=PlotData{i}(2,:);
    P3=PlotData{i}(3,:);
    P4=PlotData{i}(4,:);
    P5=PlotData{i}(5,:);
    P6=PlotData{i}(6,:);
    P7=PlotData{i}(7,:);
    P8=PlotData{i}(8,:);
    plotState(P1,P2,P3,P4,P5,P6,P7,P8);
    DrawCplrPath(CplrPath);
    gif
end
end

%Alternate CONSTRAINTS FUNCTIONS
function [cost]= constraintAlt (initConst, knownPts, x)
P1=knownPts(1,:);
P2=knownPts(2,:);
P3=knownPts(3,:);
P6=knownPts(4,:);
P4=x(1:3);
P5=x(4:6);
P7=x(7:9);
P8=x(10:12);

%Link2
c21=initConst(1)-RRconstAlt(P2,P4);
c22=initConst(2)-RRconstAlt(P2,P5);
c23=initConst(3)-RRconstAlt(P4,P5);
%Link3
c3=initConst(4)-RRconstAlt(P4,P6);
%Link4
c4=initConst(5)-RRconstAlt(P3,P7);
%Link5
c51=initConst(6)-RRconstAlt(P5,P7);
c52=initConst(7)-RRconstAlt(P5,P8);
c53=initConst(8)-RRconstAlt(P7,P8);

c=[c21, c22, c23, c3, c4, c51, c52, c53];
cost=norm(c)^2+(1-norm(P4))^2+(1-norm(P5))^2+(1-norm(P7))^2+(1-norm(P8))^2;
end
function [cost]= RRconstAlt (P1,P2)
cost=norm(P1-P2);
end

%PLOT MECHANISM STATE
function []= plotState(P1,P2,P3,P4,P5,P6,P7,P8)

cla
DrawSphere([0,0,0],1);
hold on

link_color='k';
patch_color=[.8,1,.5];

%Link 1(PRR)
Prism_jt=(PlanePt2NearestPt(P2,P1)+PlanePt2NearestPt(P3,P1))/2;
Prism_jt=Prism_jt/norm(Prism_jt);
P_jt=DrawPrismJoint(Prism_jt,P1);
colorSpLink(P_jt,P2,P3,patch_color,1.01);
DrawSpLink(P_jt,P2,link_color,1.01);
DrawSpLink(P_jt,P3,link_color,1.01);
DrawSpLink(P2,P3,link_color,1.01);

%Link 2
colorSpLink(P2,P4,P5,patch_color,1.02);
DrawSpLink(P2,P4,link_color,1.02);
DrawSpLink(P2,P5,link_color,1.02);
DrawSpLink(P4,P5,link_color,1.02);
%Link 3
DrawSpLink(P4,P6,link_color,1.01);
%Link 4
DrawSpLink(P3,P7,link_color,1.01);
%Link 5
colorSpLink(P5,P7,P8,patch_color,1.01);
DrawSpLink(P5,P7,link_color,1.01);
DrawSpLink(P5,P8,link_color,1.01);
DrawSpLink(P7,P8,link_color,1.01);
plot3(0,0,0,'*');

%Prismatic Joints
plotCircle3D([0,0,0],P1,1,[139,69,19]/255)
%Revolute Joints
DrawMovingPivot(P2)
DrawMovingPivot(P3)
DrawMovingPivot(P4)
DrawMovingPivot(P5)
DrawMovingPivot(P7)
%Fixed Joints
DrawFixedPivot(P6)
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
function []= DrawSpLink(Pt1, Pt2, col, radius)
perturb=radius;
pt1=perturb*Pt1/norm(Pt1);
pt2=perturb*Pt2/norm(Pt2);
pts=[];
iter=50;
for i=0:iter
    pt=pt1+i*(pt2-pt1)/iter;
    pts(end+1,:)=perturb*pt/norm(pt);
end
plot3(pts(:,1),pts(:,2),pts(:,3),'linewidth',1,'color',col);
end
function []=colorSpLink(P1,P2,P3,col,radius)
perturb=radius;
p1=perturb*P1/norm(P1);
p2=perturb*P2/norm(P2);
p3=perturb*P3/norm(P3);
iter=50;
pts=[];

for i=0:iter
    pt=p1+i*(p2-p1)/iter;
    pts(end+1,:)=perturb*pt/norm(pt);
end
for i=0:iter
    pt=p2+i*(p3-p2)/iter;
    pts(end+1,:)=perturb*pt/norm(pt);
end
for i=0:iter
    pt=p3+i*(p1-p3)/iter;
    pts(end+1,:)=perturb*pt/norm(pt);
end
patch(pts(:,1),pts(:,2),pts(:,3),col);
alpha(.7)
end
function plotCircle3D(center,normal,radius,color)
% center is an array [x,y,z]
% normal is an array [nx,ny,nz]
% radius is a value
% E.g., plotCircle3D([0,0,0],[-1,0,0],1)
theta=0:0.01:2*pi;
v=null(normal);
points=repmat(center',1,size(theta,2))+radius*(v(:,1)*cos(theta)+v(:,2)*sin(theta));
plot3(points(1,:),points(2,:),points(3,:),'color',color,'LineStyle',':','linewidth',2);
end
function [N_pt]=PlanePt2NearestPt(Pt,Pl)
t=(Pt(1)*Pl(1)+Pt(2)*Pl(2)+Pt(3)*Pl(3))/norm(Pl)^2;
N_pt=Pt-t*Pl;
end
function [attachPt]=DrawPrismJoint(Pt,Circ)
P=Pt/norm(Pt);
C=Circ/norm(Circ);
eps1=pi/30;
Prism1=rodrigues_rot(P,C,eps1);
Prism2=rodrigues_rot(P,C,-eps1);

eps2=.04*Circ;
PR1=Prism1+eps2; PR1=PR1/norm(PR1);
PR2=Prism2+eps2; PR2=PR2/norm(PR2);
PL1=Prism1-eps2; PL1=PL1/norm(PL1);
PL2=Prism2-eps2; PL2=PL2/norm(PL2);
DrawSpLink(PR1, PR2, 'k',1.01);
DrawSpLink(PL1, PL2, 'k',1.01);
attachPt=(PR1+PR2)/2; attachPt=attachPt/norm(attachPt);
end
function v_rot = rodrigues_rot(v,k,theta)
    [m,n] = size(v);
    if (m ~= 3 && n ~= 3)
        error('input vector is/are not three dimensional'), end
    if (size(v) ~= size(k)) 
        error('rotation vector v and axis k have different dimensions'),end
    
    k = k/sqrt(k(1)^2 + k(2)^2 + k(3)^2); % normalize rotation axis
    No = numel(v)/3; % number of vectors in array
    v_rot = v; % initialize rotated vector array
    if ( n == 3 )
        crosskv = v(1,:); % initialize cross product k and v with right dim.
        for i = 1:No
            crosskv(1) = k(2)*v(i,3) - k(3)*v(i,2);
            crosskv(2) = k(3)*v(i,1) - k(1)*v(i,3); 
            crosskv(3) = k(1)*v(i,2) - k(2)*v(i,1);
            v_rot(i,:) = cos(theta)*v(i,:) + (crosskv)*sin(theta)...
                            + k*(dot(k,v(i,:)))*(1 - cos(theta));
        end
    else % if m == 3 && n ~= 3
        crosskv = v(:,1); % initialize cross product k and v with right dim.
        for i = 1:No
            crosskv(1) = k(2)*v(3,i) - k(3)*v(2,i);
            crosskv(2) = k(3)*v(1,i) - k(1)*v(3,i); 
            crosskv(3) = k(1)*v(2,i) - k(2)*v(1,i);
            v_rot(:,i) = cos(theta)*v(:,i) + (crosskv)*sin(theta)...
                            + k*(dot(k,v(:,i)))*(1 - cos(theta));
        end
    end
end
function DrawMovingPivot(Pt)
r = .04;
[x,y,z] = sphere(10);
x0 = Pt(1); y0 = Pt(2); z0 = Pt(3);
x = x*r + x0;
y = y*r + y0;
z = z*r + z0;
surface(x,y,z,'FaceAlpha',1,'FaceColor', 'k','EdgeColor','none')
end
function DrawFixedPivot(Pt)
r = .075;
[x,y,z] = sphere(10);
x0 = Pt(1); y0 = Pt(2); z0 = Pt(3);
x = x*r + x0;
y = y*r + y0;
z = z*r + z0;
color=[139,69,19]/255;
surface(x,y,z,'FaceAlpha',1,'FaceColor', color,'EdgeColor','none')
end
function DrawCplrPath(Pt)
plot3(Pt(:,1),Pt(:,2),Pt(:,3),'-','Color',[.16,.45,.75,.7],'LineWidth',3);
end