function [] = PlotLinkage(PlotData,CplrPath)
% Visualize the mechanism
% PlotData consist of Link data and Lines Data
figure(1)
hold on

Links=PlotData.Links;
nLinks=length(Links);
for i=1:nLinks
    if (length(Links{i})==2)
        drawLink(Links{i}(1,:),Links{i}(2,:),.1);
        drawRev(Links{i}(1,:));
        drawRev(Links{i}(2,:));
    else
        drawLink(Links{i}(1,:),Links{i}(end-1,:),.1);
        drawRev(Links{i}(1,:));
        drawRev(Links{i}(2,:));
    end
    %plot(Links{i}(:,1),Links{i}(:,2));
    %fill(Links{i}(:,1),Links{i}(:,2),'b','FaceAlpha','.1')
end

Lines=PlotData.Lines;
[nLines,~]=size(Lines);
for i=1:nLines
    L=Lines(i,:);
    a=L(1);
    b=L(2);
    c=L(3);
    if (b==0)
        pt1=[-c/a,10];
        pt2=[-c/a,-10];
    else
        pt1=[10,-(a*10+c)/b];
        pt2=[-10,-(a*-10+c)/b];
    end
    plot([pt1(1),pt2(1)],[pt1(2),pt2(2)],'-.');
end

drawFP(Links{1}(1,:),.1);
drawFP([3.89, 2.25],.1);
drawFP(Links{7}(2,:),.1);
drawP(Links{9}(1,:),.15);
drawCouplerPath(CplrPath);

hold off
drawnow

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
%Draw coupler path
function [] = drawCouplerPath(path)
plot(path(:,1),path(:,2),'-','Color',[.16,.45,.75,.7],'LineWidth',3);
end




