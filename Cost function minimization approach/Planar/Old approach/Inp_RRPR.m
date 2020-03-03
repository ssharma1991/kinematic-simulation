function [initData]= Inp_RRPR()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[0, 0], 'input'}
    {'R',[0, 1]}
    {'P',[4, 1, 135]}
    {'R',[3, 0]}};

initData.Links={{[1,2]}
    {[2,3]}
    {[3,4]}
    {[1,4],'G'}};

xlim([-2 6]) 
ylim([-4 4])
end