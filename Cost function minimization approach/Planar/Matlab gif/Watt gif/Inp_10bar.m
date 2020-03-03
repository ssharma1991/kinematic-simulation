function [initData]= Inp_10bar()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[1.9, .7], 'input'}
    {'R',[1.6, 1.2]}
    {'R',[1.94, 2.73]}
    {'R',[3.89, 2.25]}
    {'R',[4.85, 2]}
    {'R',[5.6, 1.9]}
    {'R',[4.85, 1.6]}
    {'R',[5.6, 1.42]}
    {'R',[6, 1.9]}
    {'R',[5.6, -1.2]}
    {'P',[5.6, -1.2, 90]}};

initData.Links={{[1,2]}
    {[2,3]}
    {[3,4,5,6]}
    {[5,7]}
    {[6,8]}
    {[7,8]}
    {[7,9]}
    {[8,10]}
    {[10,11]}
    {[1,4,9,11],'G'}};

xlim([-5 7]) 
ylim([-5 5])
end