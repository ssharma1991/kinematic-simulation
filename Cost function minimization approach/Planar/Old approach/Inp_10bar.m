function [initData]= Inp_10bar()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[0, 0], 'input'}
    {'P',[-1, 3, 30]}
    {'R',[4.6, 2.5]}
    {'R',[1, 2]}
    {'R',[2, 1]}
    {'R',[5, 3]}
    {'R',[5, -2]}
    {'R',[3, -3]}
    {'P',[-1, -3, -10]}
    {'R',[-1, -3]}
    {'P',[3, 0, -75]}
    {'R',[3, 0]}
    {'P',[0, -2, -45]}};

initData.Links={{[1,2,9]}
    {[2,3,4]}
    {[4,5,6]}
    {[6,7,8]}
    {[5,11,13]}
    {[3,12]}
    {[11,12]}
    {[8,10]}
    {[9,10]}
    {[1,7,13],'G'}};

xlim([-5 7]) 
ylim([-5 5])
end