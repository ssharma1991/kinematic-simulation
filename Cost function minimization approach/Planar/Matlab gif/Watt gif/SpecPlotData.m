function [PlotData] = SpecPlotData(initSimData,newSimData,initSpecPlotData)
% Gives Links and Lines data from Simulation and Specification Data

% Remove ground link
for i=1:length(newSimData.Links)
    if (length(newSimData.Links{i})==2)
        newSimData.Links(i)=[];
        initSimData.Links(i)=[];
    end
end

LinksPosOrient=SimParam2LinkPosOrient(newSimData);
prevLinksPosOrient=SimParam2LinkPosOrient(initSimData);

Links={};
for i=1:length(newSimData.Links)
    dtheta=LinksPosOrient(i,3)-prevLinksPosOrient(i,3);
    T1=[1,0,-prevLinksPosOrient(i,1);0,1,-prevLinksPosOrient(i,2);0,0,1];
    R=[cos(dtheta),-sin(dtheta),0;sin(dtheta),cos(dtheta),0;0,0,1];
    T2=[1,0,LinksPosOrient(i,1);0,1,LinksPosOrient(i,2);0,0,1];
    Trans=T2*R*T1;
    pts=initSpecPlotData.Links{i};
    pts(:,3)=1;
    newpts=Trans*pts';
    Links{i}=newpts(1:2,:)';
end

Lines=[];
for i=1:length(newSimData.Joints)
    % All P joints
    if isequal(newSimData.Joints{i}{1},'P')
        Li=newSimData.Joints{i}{2};
        Lines(end+1,:)=Li;
    end
end
PlotData.Links=Links;
PlotData.Lines=Lines;
end

function [LinkPosOrient]= SimParam2LinkPosOrient(simData)
Joints=simData.Joints;
Links=simData.Links;
nLinks=length(Links);
LinkPosOrient=zeros(nLinks,3);
for i=1:nLinks
    Jts=Links{i}{1};
    Jt1=Joints{Jts(1)};
    Jt2=Joints{Jts(2)};
    if isequal(Jt1{1},'R') && isequal(Jt2{1},'R')
        LinkPosOrient(i,1:2)=Jt1{2};
        LinkPosOrient(i,3)=atan2(Jt2{2}(2)-Jt1{2}(2), Jt2{2}(1)-Jt1{2}(1));
    elseif isequal(Jt1{1},'R') && isequal(Jt2{1},'P')
        LinkPosOrient(i,1:2)=Jt1{2};
        pt=linePt2NearestPt(Jt1{2},Jt2{2});
        LinkPosOrient(i,3)=atan2(pt(2)-Jt1{2}(2), pt(1)-Jt1{2}(1));
    elseif isequal(Jt1{1},'P') && isequal(Jt2{1},'R')
        LinkPosOrient(i,1:2)=Jt2{2};
        pt=linePt2NearestPt(Jt2{2},Jt1{2});
        LinkPosOrient(i,3)=atan2(Jt2{2}(2)-pt(2), Jt2{2}(1)-pt(1));
    elseif isequal(Jt1{1},'P') && isequal(Jt2{1},'P')
        %If parallel lines, degenerate condition
        LinkPosOrient(i,1:2)=lineLineIntersection(Jt1{2},Jt2{2});
        LinkPosOrient(i,3)=atan2(-Jt1{2}(1),Jt1{2}(2));
    end
end
end
function [pt]= lineLineIntersection(L1,L2)
%Not defined for paralled lines
x=-(L1(3)*L2(2)-L2(3)*L1(2))/(L1(1)*L2(2)-L2(1)*L1(2));
if (L1(2)==0)
    y=-(L2(1)*x+L2(3))/L2(2);
else
    y=-(L1(1)*x+L1(3))/L1(2);
end
pt=[x,y];
end
function [nearestPt]= linePt2NearestPt(P,L)
a=L(1);
b=L(2);
c=L(3);
x=P(1);
y=P(2);
nearestPt(1)=(b*(b*x-a*y)-a*c)/(a^2+b^2);
nearestPt(2)=(a*(-b*x+a*y)-b*c)/(a^2+b^2);
end
