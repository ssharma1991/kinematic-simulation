function [initData]= Inp_RRRP()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[0, 0], 'input'}
    {'R',[0, 1]}
    {'R',[4, 2]}
    {'P',[4, 0, 60]}};

initData.Links={{[1,2]}
    {[2,3]}
    {[3,4]}
    {[1,4],'G'}};

xlim([-2 6]) 
ylim([-2 3])
end

