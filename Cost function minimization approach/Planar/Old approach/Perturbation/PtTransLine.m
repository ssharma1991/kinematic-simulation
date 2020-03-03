clc
clear all
close all

line=[1,2,3];
Pt=[10,10];
plot(Pt(1),Pt(2),'o');
hold on
PlotLine(line(1),line(2),line(3));
axis([-20 20 -20 20])
% translate the point along the line
for i=1:10
    n_pt=Pt+i*[line(2),-line(1)]
    plot(n_pt(1),n_pt(2),'o');
end

function PlotLine(a,b,c)
if (b==0)
    pt1=[-c/a,10];
    pt2=[-c/a,-10];
else
    pt1=[10,-(a*10+c)/b];
    pt2=[-10,-(a*-10+c)/b];
end
plot([pt1(1),pt2(1)],[pt1(2),pt2(2)],'-.');
end