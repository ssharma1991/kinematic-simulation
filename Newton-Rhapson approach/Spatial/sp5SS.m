% Simulate a 5ss-mechanism
clc
clear all
close all

INDEX=4480;

%[FP1,FP2,FP3,FP4,FP5,MP1,MP2,MP3,MP4,MP5,C]
Pts=[[-7.20078633 -5.2930031   7.87116744]
 [ 5.71803324 -0.71371165 -8.25749538]
 [-8.74711427 -4.59885719  5.48763714]
 [-9.99593062 -8.72040051  6.08658925]
 [-3.87891519  6.61366631  8.91117982]
 [-6.60806082 -9.9868523   3.93482383]
 [ 8.19190297 -9.04651467  6.06662798]
 [ 7.83840293 -5.73238752 -0.61694473]
 [ 4.14722147 -0.48122018 -2.04096393]
 [-0.97390133 -9.04023547  1.92449292]
 [ 2.19533097 -7.1743553   5.35866566]];

simPts=simulate5SS(Pts);
cplr_Path=squeeze(simPts(11,:,:))';
createGIF(simPts, cplr_Path);
cla
drawMech(Pts,cplr_Path)

function [sim,cplr] = simulate5SS(Pts)
simP=simulate5SS_linearActuation(Pts, .1);
simN=simulate5SS_linearActuation(Pts, -.1);
sim=cat(3,flip(simN,3),simP);
end
function [simPts] = simulate5SS_linearActuation(Pts, iter)
DistConsts=[1,6; 2,7; 3,8; 4,9; 5,10; 
    11,6; 11,7; 11,8; 11,9; 11,10; 
    6,7; 6,8; 6,9; 6,10; 
    7,8; 7,9; 7,10; 
    1,7];

L_init=LenConst(Pts,DistConsts);
L_target=L_init;
disp=0;
simPts=[];
while(true)
    L_target(end)=L_init(end)+disp;
    Pts=NewtonRhapson(L_target,Pts,DistConsts);
    
    F=Cost(LenConst(Pts,DistConsts),L_target);
    if norm(F)>10^-8
        break
    end
    disp=disp+iter;
    simPts=cat(3,simPts,Pts);
    %cplr_Path=[cplr_Path;Pts(11,:)];
    
    %cla
    %drawMech(Pts,cplr_Path);
    %drawnow
end
end


% SIMULATION Functions
function [Pts]= NewtonRhapson(L_target,Pts,DistConsts)
F=Cost(LenConst(Pts,DistConsts),L_target);
for i=1:10
    F_dash=CostGradient(Pts,DistConsts);
    dX=-F_dash\F;
    dX=reshape(dX,[3,6])';
    Pts(6:11,:)=Pts(6:11,:)+dX;
    F=Cost(LenConst(Pts,DistConsts),L_target);
    if (norm(F)<10^-8)
        break
    end
end
end
function [Cst]= Cost(CurrentL,TargetL)
Cst=(CurrentL-TargetL);
end
function [grad]= CostGradient(Pts,DistConsts)
grad=zeros(18,18);
for i=1:18
    % Permute over each constraint
    Pt1_index=DistConsts(i,1);
    Pt2_index=DistConsts(i,2);
    Pt1=Pts(Pt1_index,:);
    Pt2=Pts(Pt2_index,:);
    
    %Partial derivative wrt x,y,z of first coordinate
    if Pt1_index>5
        dx=(Pt1(1)-Pt2(1))/norm(Pt1-Pt2);
        dy=(Pt1(2)-Pt2(2))/norm(Pt1-Pt2);
        dz=(Pt1(3)-Pt2(3))/norm(Pt1-Pt2);
        
        grad(i,3*(Pt1_index-5)-2)=dx;
        grad(i,3*(Pt1_index-5)-1)=dy;
        grad(i,3*(Pt1_index-5))=dz;
    end
    
    %Partial derivative wrt x,y,z of second coordinate
    dx=-(Pt1(1)-Pt2(1))/norm(Pt1-Pt2);
    dy=-(Pt1(2)-Pt2(2))/norm(Pt1-Pt2);
    dz=-(Pt1(3)-Pt2(3))/norm(Pt1-Pt2);
        
    grad(i,3*(Pt2_index-5)-2)=dx;
    grad(i,3*(Pt2_index-5)-1)=dy;
    grad(i,3*(Pt2_index-5))=dz;
end
end
function [Lengths]= LenConst(Pts,DistConsts)
n_const=length(DistConsts);
for i=1:n_const
    Pt1_index=DistConsts(i,1);
    Pt2_index=DistConsts(i,2);
    Pt1=Pts(Pt1_index,:);
    Pt2=Pts(Pt2_index,:);
    Lengths(i)=norm(Pt1-Pt2);
end
Lengths=Lengths';
end

% DRAWING function
function []= createGIF(simPts, cplr_Path)

for i=1:length(cplr_Path)
    cla
    Pts=squeeze(simPts(:,:,i));
    drawMech(Pts,cplr_Path)
    drawnow
    
    if i==1
        gif('sp5SS.gif');
    else
        gif
    end
end

for i=length(cplr_Path):-1:1
    cla
    Pts=squeeze(simPts(:,:,i));
    drawMech(Pts,cplr_Path)
    drawnow
    gif
end

length(cplr_Path)
end
function []= drawMech(Pts,cplr_Path)
dyads=[Pts(1:5,:),Pts(6:10,:)];
cplr=Pts(11,:);

plot3(0,0,0,'*')
grid on
hold on

%Print SS Dyads
for i=1:5
    x=[dyads(i,1),dyads(i,4)];
    y=[dyads(i,2),dyads(i,5)];
    z=[dyads(i,3),dyads(i,6)];
    plot3(x,y,z,'b','LineWidth',2);
end

%Print Coupler
for i=1:5
    x=[dyads(i,4),cplr(1)];
    y=[dyads(i,5),cplr(2)];
    z=[dyads(i,6),cplr(3)];
    plot3(x,y,z,'g','LineWidth',2)
end

%Print Actuation link
x=[dyads(1,1),dyads(2,4)];
y=[dyads(1,2),dyads(2,5)];
z=[dyads(1,3),dyads(2,6)];
plot3(x,y,z,':b','LineWidth',2);

%Print Fixed pivots
for i=1:5
    scatter3(dyads(i,1),dyads(i,2),dyads(i,3),8,'k^','LineWidth',2)
end

%Print Moving pivots
for i=1:5
    scatter3(dyads(i,4),dyads(i,5),dyads(i,6),2,'bo','LineWidth',2)
end

%Print Coupler point
scatter3(cplr(1),cplr(2),cplr(3),20,'ko','LineWidth',1)
axis ([-15 10 -25 19 -15 10])
pbaspect([1 1 1])

%Print Coupler Path
plot3(cplr_Path(:,1),cplr_Path(:,2),cplr_Path(:,3),'r','LineWidth',2);
end
