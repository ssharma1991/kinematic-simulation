function [initData]= Inp_RRRR()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[0, 0], 'input'}
    {'R',[0, 1]}
    {'R',[1, 2]}
    {'R',[2, 0]}};

initData.Links={{[1,2]}
    {[2,3]}
    {[3,4]}
    {[1,4],'G'}};

xlim([-2 4]) 
ylim([-2 4])
end
