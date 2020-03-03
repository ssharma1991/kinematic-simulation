function [simData]= Spec2SimParam (specParam)
% SPECIFICATION parameters
% Revolute Joint (x,y)
% Prismatic Joint (x,y,angle)
%
% SIMULATION parameters
% Revolute Joint =Point (x,y)
% Prismatic Joint =Line (a,b,c)

Joints=specParam.Joints;

n_joints=length(Joints);
for i=1:n_joints
    Joint=Joints{i};
    if isequal(Joint{1},'P')
        x=Joint{2}(1);
        y=Joint{2}(2);
        ang=Joint{2}(3);
        a=-sind(ang);
        b=cosd(ang);
        c=-(a*x+b*y);
        Joints{i}{2}=[a,b,c];
    end
end

simData.Joints=Joints;
simData.Links=specParam.Links;
end