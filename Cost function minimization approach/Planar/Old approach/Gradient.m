function [grad]= Gradient(Data, MechConst, stateIndex)
% CALCULATE GRADIENT

nSt=length(stateIndex);
grad=[];
for i=1:nSt
    jtIndex=stateIndex{i}{1};
    if isequal(stateIndex{i}{2},'R')
        gradx=GradientConst(Data, MechConst, jtIndex, 1);
        grady=GradientConst(Data, MechConst, jtIndex, 2);
        grad(end+1:end+2)=[gradx, grady];
    elseif isequal(stateIndex{i}{2},'P')
        grada=GradientConst(Data, MechConst, jtIndex, 1);
        gradb=GradientConst(Data, MechConst, jtIndex, 2);
        gradc=GradientConst(Data, MechConst, jtIndex, 3);
        grad(end+1:end+3)=[grada, gradb, gradc];
    end
end
end
function [xigrad]= GradientConst(Data, MechConst, jointIndex, coordIndex)
% Gradiend for each state index
nLinks=length(Data.Links);
xigrad=0;
for i=1:nLinks
    isConstLink=~isempty(find(Data.Links{i}{1} == jointIndex));
    if (isConstLink)
        xigrad=xigrad+LinkGradConst(Data,MechConst,i,jointIndex, coordIndex);
    end
end
end
function [link_grad]= LinkGradConst(Data, MechConst, linkIndex, jointIndex, coordIndex)
Jts=Data.Links{linkIndex}{1};
nJts=length(Jts);
link_grad=0;
for i=1:nJts-1
    jt_first= Data.Joints{Jts(i)};
    for j=i+1:nJts
        jt_second= Data.Joints{Jts(j)};
        if (Jts(i)==jointIndex)
            if (isequal(jt_first{1},'R') && isequal(jt_second{1},'R'))
                link_grad=link_grad+RRConstFirstOrder(MechConst{linkIndex}(i,j),jt_first{2},jt_second{2},coordIndex);
            elseif (isequal(jt_first{1},'R') && isequal(jt_second{1},'P'))
                link_grad=link_grad+RPConstFirstOrder(MechConst{linkIndex}(i,j),jt_first{2},jt_second{2},1,coordIndex);
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'R'))
                link_grad=link_grad+RPConstFirstOrder(MechConst{linkIndex}(i,j),jt_second{2},jt_first{2},2,coordIndex);
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'P'))
                link_grad=link_grad+PPConstFirstOrder(MechConst{linkIndex}(i,j),jt_first{2},jt_second{2},coordIndex);
            end
        elseif (Jts(j)==jointIndex)
            if (isequal(jt_first{1},'R') && isequal(jt_second{1},'R'))
                link_grad=link_grad+RRConstFirstOrder(MechConst{linkIndex}(i,j),jt_second{2},jt_first{2},coordIndex);
            elseif (isequal(jt_first{1},'R') && isequal(jt_second{1},'P'))
                link_grad=link_grad+RPConstFirstOrder(MechConst{linkIndex}(i,j),jt_first{2},jt_second{2},2,coordIndex);
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'R'))
                link_grad=link_grad+RPConstFirstOrder(MechConst{linkIndex}(i,j),jt_second{2},jt_first{2},1,coordIndex);
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'P'))
                link_grad=link_grad+PPConstFirstOrder(MechConst{linkIndex}(i,j),jt_second{2},jt_first{2},coordIndex);
            end
        end
    end
end
%loop to calc constraint (RR,RP,PP) grad between joints
%selection of equation for specific const.
end
function [grad]= RRConstFirstOrder (dist,P1,P2,coordIndex)
if (coordIndex==1)
    grad=2*(P2(1)-P1(1))*(dist-norm(P2-P1))/norm(P2-P1);
elseif (coordIndex==2)
    grad=2*(P2(2)-P1(2))*(dist-norm(P2-P1))/norm(P2-P1);
end
end
function [grad]= RPConstFirstOrder (dist,P,L,JtIndex,coordIndex)
Val=L(1)*P(1)+L(2)*P(2)+L(3);
if abs(Val)==0
    AbsDiff=0;
elseif Val>0
    AbsDiff=1;
else
    AbsDiff=-1;
end

if (JtIndex==1 && coordIndex==1)
    grad=-2*L(1)*(dist-abs(Val)/norm([L(1),L(2)]))*AbsDiff/norm([L(1),(2)]);
elseif (JtIndex==1 && coordIndex==2)
    grad=-2*L(2)*(dist-abs(Val)/norm([L(1),L(2)]))*AbsDiff/norm([L(1),L(2)]);
elseif (JtIndex==2 && coordIndex==1)
    grad=2*(dist-abs(Val)/norm([L(1),L(2)]))*(L(1)*abs(Val)/norm([L(1),L(2)])^3-P(1)*AbsDiff/norm([L(1),L(2)]));
elseif (JtIndex==2 && coordIndex==2)
    grad=2*(dist-abs(Val)/norm([L(1),L(2)]))*(L(2)*abs(Val)/norm([L(1),L(2)])^3-P(2)*AbsDiff/norm([L(1),L(2)]));
elseif (JtIndex==2 && coordIndex==3)
    grad=-2*(dist-abs(Val)/norm([L(1),L(2)]))*AbsDiff/norm([L(1),L(2)]);
end
end
function [grad]= PPConstFirstOrder (dist,L1,L2,coordIndex)
val1=L1(1)*L2(1)+L1(2)*L2(2);
val2=norm([L1(1),L1(2)]);
val3=norm([L2(1),L2(2)]);
if (coordIndex==1)
    grad=2*(L1(1)*val1/(val2^3*val3)-L2(1)/(val2*val3))*(dist-val1/(val2*val3));
elseif (coordIndex==2)
    grad=2*(L1(2)*val1/(val2^3*val3)-L2(2)/(val2*val3))*(dist-val1/(val2*val3));
elseif (coordIndex==3)
    grad=0;
end
end

