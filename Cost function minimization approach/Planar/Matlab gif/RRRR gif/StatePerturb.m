function [newState]= StatePerturb (State, ang)
% PERTURB INPUT LINK- update joints on input link

perturb_ang=ang;
input=State.input;
perturbed=State.perturbed;

for i=1:length(perturbed)
    InpGrndPivot=input{3};
    T1=[1,0,-InpGrndPivot(1);0,1,-InpGrndPivot(2);0,0,1];
    R=[cos(ang),-sin(ang),0;sin(ang),cos(ang),0;0,0,1];
    T2=[1,0,InpGrndPivot(1);0,1,InpGrndPivot(2);0,0,1];
    Trans=T2*R*T1;
    if isequal(perturbed{i}{2},'R')
        InpMovPivot=perturbed{i}{3};
        InpMovPivot(3)=1;
        NewMovPivot=Trans*InpMovPivot';
        perturbed{i}{3}=NewMovPivot(1:2)';
    elseif isequal(perturbed{i}{2},'P')
        InpMovLine=perturbed{i}{3};
        NewMovLine=InpMovLine*inv(Trans);
        perturbed{i}{3}=NewMovLine(1:3);
    end
end

newState=State;
newState.perturbed=perturbed;
end