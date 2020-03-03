function [SpecPlotData] = initSpecPlotData(inpSpecData)
% Gives Links and Lines data 

Links={};
for i=1:length(inpSpecData.Links)
    % All except ground link
    if (length(inpSpecData.Links{i})~=2)
        Linki=getLinkPlotData(inpSpecData,i);
        Links{end+1}=Linki;
    end
end

Lines=[];
for i=1:length(inpSpecData.Joints)
    % All P joints
    if isequal(inpSpecData.Joints{i}{1},'P')
        Li=getLinePlotData(inpSpecData.Joints{i}{2});
        Lines(end+1,:)=Li;
    end
end

SpecPlotData.Links=Links;
SpecPlotData.Lines=Lines;
end

function [LinkData] = getLinkPlotData(SpecData,LinkIndex)
Jts=SpecData.Links{LinkIndex}{1};
nJts=length(Jts);
pts=[];
for i=1:nJts
    jt=SpecData.Joints{Jts(i)};
    if isequal(jt{1},'R')
        pts(end+1,1:2)=jt{2};
    else
        pts(end+1,1:2)=jt{2}(1:2);
    end
end

if length(pts)==2
    plotptsindex=[1,2];
else
    plotptsindex=convhull(pts(:,1),pts(:,2));
end
LinkData=pts(plotptsindex,:);
end
function [LineData] = getLinePlotData(Line)
x=Line(1);
y=Line(2);
ang=Line(3);
a=-sind(ang);
b=cosd(ang);
c=-(a*x+b*y);
LineData=[a,b,c];
end
