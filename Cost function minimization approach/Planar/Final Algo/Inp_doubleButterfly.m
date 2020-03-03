function [initData]= Inp_doubleButterfly()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[5,0],'input'}
    {'R',[5, 2.5]}
    {'R',[0, 4]}
    {'R',[-5, 2]}
    {'R',[-5, 0]}
    {'R',[2, 3]}
    {'R',[-1, 2]}
    {'R',[1, .5]}
    {'R',[0, 3]}
    {'R',[0, 0]}};

initData.Links={{[1,2]}
    {[2,3,6]}
    {[5,4]}
    {[4,3,7]}
    {[6,8]}
    {[7,9]}
    {[8,9,10]}
    {[1,5,10],'G'}};

xlim([-7 7]) 
ylim([-4 8])
end