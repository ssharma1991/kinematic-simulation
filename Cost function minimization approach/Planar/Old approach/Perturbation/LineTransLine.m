clc
clear all
close all

Fline=[1,2,3];
Fline=Fline/norm([Fline(1),Fline(2)]);
Mline=[-1,2,1];
PlotLine(Fline(1),Fline(2),Fline(3));
hold on
PlotLine(Mline(1),Mline(2),Mline(3));
axis([-10 10 -10 10])
% translate the point along the line
for i=1:10
    n_line=Mline+i*[0,0,-1]*(Fline(1)*Mline(2)-Fline(2)*Mline(1));
    PlotLine(n_line(1),n_line(2),n_line(3));
    a1=Fline(1);b1=Fline(2);c1=Fline(3);
    a2=n_line(1);b2=n_line(2);c2=n_line(3);
    y=(-c2*a1+c1*a2)/(a1*b2-a2*b1);
    x=-(b1*y+c1)/a1;
    plot(x,y,'*');
    [x,y]
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