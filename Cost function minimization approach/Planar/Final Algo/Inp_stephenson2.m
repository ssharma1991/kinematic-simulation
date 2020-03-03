function [initData]= Inp_stephenson2()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[0, -1], 'input'}
    {'R',[1, .5]}
    {'P',[2, 4.7, 10]}
    {'R',[3.25, 1.4]}
    {'R',[7.72, 1.44]}
    {'R',[11.66, 4.17]}
    {'P',[10.08, -1.24, 0]}
    {'R',[6, -2]}};

initData.Links={{[1,2]}
    {[2,3,4]}
    {[3,6]}
    {[4,5,8]}
    {[5,6,7]}
    {[1,7],'G'}};

xlim([-5 15]) 
ylim([-5 15])
end