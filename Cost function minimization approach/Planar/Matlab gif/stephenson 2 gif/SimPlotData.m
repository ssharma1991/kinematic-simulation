function [PlotData] = SimPlotData(SimData)
% Gives Links and Lines data from Simulation Data

Links={};
for i=1:length(SimData.Links)
    % All except ground link
    if (length(SimData.Links{i})~=2)
        Linki=getLinkPlotData(SimData,LinkIndex);
        Links{end+1}=Linki;
    end
end

Lines=[];
for i=1:length(SimData.Joints)
    % All P joints
    if isequal(SimData.Joints{i}{1},'P')
        Li=SimData.Joints{i}{2};
        Lines(end+1,:)=Li;
    end
end

PlotData.Links=Links;
PlotData.Lines=Lines;
end
function [LinkData] = getLinkPlotData(SimData,LinkIndex)
Jts=SimData.Links{LinkIndex}{1};
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
LinkData=allpts(plotptsindex,:);
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