function [initData]= Inp_jansons()
% INPUT / SPECIFICATION paremeters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
% Links (Jt1, Jt2, ...)

initData.Joints={{'R',[2.77, 2.31], 'input'}
    {'R',[2.17, 3.33]}
    {'R',[-1.48, 4.69]}
    {'R',[.66, -1.3]}
    {'R',[-.22, 1.72]}
    {'R',[-3.17, .66]}
    {'R',[-2.08, -2.24]}
    {'R',[2.54, -4.64]}};

initData.Links={{[1,2]}
    {[2,3]}
    {[2,4]}
    {[3,5,6]}
    {[5,4]}
    {[6,7]}
    {[4,7,8]}
    {[1,5],'G'}};

xlim([-7 5]) 
ylim([-6 6])
end