function [] = PlotLinkage(PlotData)
% Visualize the mechanism
% PlotData consist of Link data and Lines Data
figure(1)
hold on

Links=PlotData.Links;
nLinks=length(Links);
for i=1:nLinks
    %plot(Links{i}(:,1),Links{i}(:,2));
    fill(Links{i}(:,1),Links{i}(:,2),'b','FaceAlpha','.1')
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

hold off
drawnow

end

