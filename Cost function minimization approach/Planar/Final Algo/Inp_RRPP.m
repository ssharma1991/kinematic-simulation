function [initData]= Inp_RRPP()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)
%Scotch-yoke

initData.Joints={{'R',[0, 0], 'input'}
    {'R',[1, 1]}
    {'P',[1, 1, 90]}
    {'P',[2, 0, 0]}};

initData.Links={{[1,2]}
    {[2,3]}
    {[3,4]}
    {[1,4],'G'}};

xlim([-2 4]) 
ylim([-2 4])
end
