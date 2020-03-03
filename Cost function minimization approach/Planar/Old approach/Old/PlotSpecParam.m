function []= PlotSpecParam (SpecData)
% PLOT- visualize the mechanism

figure(1)
%Print Links
for i=1:length(SpecData.Links)
    hold on
    %Print all links except ground
    if (length(SpecData.Links{i})~=2)
        PlotLinkSpecParam(SpecData,i);
        drawnow
    end
end

%Print Prismatic Joints
for i=1:length(SpecData.Joints)
    if isequal(SpecData.Joints{i}{1},'P')
        PlotPrismSpecParam(SpecData,i);
    end
end
hold off
end
function []= PlotLinkSpecParam (SpecData,LinkIndex)
jts=SpecData.Links{LinkIndex}{1};
plotpts=[];
nJts=length(Jts);
for i=1:nJts
    pt=SpecData.Joints{jts(i)}{2};
    if isequal(jt{1},'R')
        plotpts=cat(1,plotpts,pt(1:2));
    else
        %calc plot point for this prismatic link
        lines(end+1,1:3)=jt{2};
    end
end

plot(plotpts(:,1),plotpts(:,2));

end
function []= PlotPrismSpecParam (SpecData,JtIndex)
jt=SpecData.Joints{JtIndex}{2};
dirn=[cosd(jt(3)),sind(jt(3))];
pt1=jt(1:2)+10*dirn;
pt2=jt(1:2)-10*dirn;
plot([pt1(1),pt2(1)],[pt1(2),pt2(2)],':');
end