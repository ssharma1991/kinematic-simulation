function [specParam]= Sim2SpecParam (simData, previousSimData, prevSpecParam)
% SPECIFICATION parameters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
%
% SIMULATION parameters
% Revolute Joint =Point (x,y)
% Prismatic Joint =Line (a,b,c)

Joints=simData.Joints;
LinksPosOrient=SimParam2LinkPosOrient(simData);
prevLinksPosOrient=SimParam2LinkPosOrient(previousSimData);
nJts=length(Joints);
specParam.Joints=Joints;
specParam.Links=simData.Links;
for i=1:nJts
    links=LinksContainingJt(i,simData);
    link=links(1);
    %dx=LinksPosOrient(link,1)-prevLinksPosOrient(link,1);
    %dy=LinksPosOrient(link,2)-prevLinksPosOrient(link,2);
    dtheta=LinksPosOrient(link,3)-prevLinksPosOrient(link,3);
        
    T1=[1,0,-prevLinksPosOrient(link,1);0,1,-prevLinksPosOrient(link,2);0,0,1];
    R=[cos(dtheta),-sin(dtheta),0;sin(dtheta),cos(dtheta),0;0,0,1];
    T2=[1,0,LinksPosOrient(link,1);0,1,LinksPosOrient(link,2);0,0,1];
    Trans=T2*R*T1;
    %Trans=[cos(dtheta),-sin(dtheta),dx;sin(dtheta),cos(dtheta),dy;0,0,1];
    
    if isequal(Joints{i}{1},'R')
        Jt=prevSpecParam.Joints{i}{2};
        pt=Trans*[Jt(1);Jt(2);1];
        specParam.Joints{i}{2}=pt(1:2)';
    elseif isequal(Joints{i}{1},'P')
        Jt=prevSpecParam.Joints{i}{2};
        pt=Trans*[Jt(1);Jt(2);1];
        ang=Jt(3)+dtheta;
        specParam.Joints{i}{2}=[pt(1:2)',ang];
        
        %OR another possibility
        %theta=dtheta+pi;
        %Trans2=[cos(theta),-sin(theta),dx;sin(theta),cos(theta),dy;0,0,1];
        %pt2=Trans2*[Jt(1);Jt(2);1];
        %ang2=Jt(3)+theta;
    end
end
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

function [Lk_index]= LinksContainingJt (Jt_index,simData)
Links=simData.Links;
nLinks=length(Links);
Lk_index=[];
for i=1:nLinks
    isPresent=~isempty(find(Links{i}{1} == Jt_index));
    if isPresent
        Lk_index(end+1)=i;
    end
end
end

function [pt]= lineLineIntersection(L1,L2)
x=(L1(3)*L2(2)-L2(3)*L1(2))/(L1(1)*L2(2)-L2(1)*L1(2));
y=-(L1(1)*x+L1(3))/L1(2);
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


