function []= PlotSimParam (Data)
% PLOT- visualize the mechanism

figure(1)
for i=1:length(Data.Links)
    hold on
    if (length(Data.Links{i})~=2)
        PlotLinkSimParam(Data,i)
        drawnow
    end
end
hold off
end
function []= PlotLinkSimParam(Data,LinkIndex)
Jts=Data.Links{LinkIndex}{1};
nJts=length(Jts);
pts=[];
lines=[];
for i=1:nJts
    jt=Data.Joints{Jts(i)};
    if isequal(jt{1},'R')
        pts(end+1,1:2)=jt{2};
    else
        lines(end+1,1:3)=jt{2};
    end
end

allpts=[];
[npts,~]=size(pts);
[nlines,~]=size(lines);
for  i=1:nlines
    pt=[0,0] ;
    n=0;
    for j=1:npts
        pt=pt+linePt2NearestPt(pts(j,:),lines(i,:));
        n=n+1;
    end
    allpts(end+1,1:2)=pt/n;
end

for i=1:npts
    allpts(end+1,1:2)=pts(i,:);
end

if length(allpts)==2
    plotptsindex=[1,2];
else
    plotptsindex=convhull(allpts(:,1),allpts(:,2));
end
plotpts=allpts(plotptsindex,:);

%plot(plotpts(:,1),plotpts(:,2))
fill(plotpts(:,1),plotpts(:,2),'b','FaceAlpha','.1')
% Plot slot lines
for i=1:nlines
    a=lines(i,1);
    b=lines(i,2);
    c=lines(i,3);
    pt1=[10,-(a*10+c)/b];
    pt2=[-10,-(a*-10+c)/b];
    plot([pt1(1),pt2(1)],[pt1(2),pt2(2)],'-.');
end

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

