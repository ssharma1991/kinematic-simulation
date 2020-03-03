function [initData]= Inp_RRPR()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[0, 0], 'input'}
    {'R',[.2, 4.6]}
    {'P',[1, 6.5, 135]}
    {'R',[5.1, .2]}};

initData.Links={{[1,2]}
    {[2,3]}
    {[3,4]}
    {[1,4],'G'}};

xlim([-6 6]) 
ylim([-6 6])
end