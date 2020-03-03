function [grad]= Gradient(Data, MechConst, stateIndex, inpLinks)
% CALCULATE Jacobian matrix 
% (dimension= constraints * state variables)

nSt=length(stateIndex);
grad=[];
for i=1:nSt
    jtIndex=stateIndex{i}{1};
    if isequal(stateIndex{i}{2},'R')
        gradx=GradientConst(Data, MechConst, jtIndex, 1, inpLinks);
        grady=GradientConst(Data, MechConst, jtIndex, 2, inpLinks);
        grad=cat(2,grad,[gradx, grady]);
    elseif isequal(stateIndex{i}{2},'P')
        grada=GradientConst(Data, MechConst, jtIndex, 1, inpLinks);
        gradb=GradientConst(Data, MechConst, jtIndex, 2, inpLinks);
        gradc=GradientConst(Data, MechConst, jtIndex, 3, inpLinks);
        grad=cat(2,grad,[grada, gradb, gradc]);
    end
end
end
function [xigrad]= GradientConst(Data, MechConst, jointIndex, coordIndex, inpLinks)
% Gradiend for each state index
% length of grad vector= no. of constraints 

% run for all links except fixed and input links
nLinks=length(Data.Links);
xigrad=[];
for i=1:nLinks
    isFixedLink=(length(Data.Links{i})==2);
    isInputLink=(i==inpLinks{1});
    if ~(isFixedLink || isInputLink)
        xigrad=cat(1,xigrad,LinkGradConst(Data,MechConst,i,jointIndex, coordIndex));
    end
end
end
function [link_grad]= LinkGradConst(Data, MechConst, linkIndex, jointIndex, coordIndex)
Jts=Data.Links{linkIndex}{1};
nJts=length(Jts);
link_grad=[];
ConstIndex=0;
for i=1:nJts-1
    jt_first= Data.Joints{Jts(i)};
    for j=i+1:nJts
        jt_second= Data.Joints{Jts(j)};
        ConstIndex=ConstIndex+1;
        Const=MechConst{linkIndex}(ConstIndex);
        if (Jts(i)==jointIndex)
            if (isequal(jt_first{1},'R') && isequal(jt_second{1},'R'))
                link_grad=cat(2,link_grad,RRConstFirstOrder(Const,jt_first{2},jt_second{2},coordIndex));
            elseif (isequal(jt_first{1},'R') && isequal(jt_second{1},'P'))
                link_grad=cat(2,link_grad,RPConstFirstOrder(Const,jt_first{2},jt_second{2},1,coordIndex));
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'R'))
                link_grad=cat(2,link_grad,RPConstFirstOrder(Const,jt_second{2},jt_first{2},2,coordIndex));
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'P'))
                link_grad=cat(2,link_grad,PPConstFirstOrder(Const,jt_first{2},jt_second{2},coordIndex));
            end
        elseif (Jts(j)==jointIndex)
            if (isequal(jt_first{1},'R') && isequal(jt_second{1},'R'))
                link_grad=cat(2,link_grad,RRConstFirstOrder(Const,jt_second{2},jt_first{2},coordIndex));
            elseif (isequal(jt_first{1},'R') && isequal(jt_second{1},'P'))
                link_grad=cat(2,link_grad,RPConstFirstOrder(Const,jt_first{2},jt_second{2},2,coordIndex));
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'R'))
                link_grad=cat(2,link_grad,RPConstFirstOrder(Const,jt_second{2},jt_first{2},1,coordIndex));
            elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'P'))
                link_grad=cat(2,link_grad,PPConstFirstOrder(Const,jt_second{2},jt_first{2},coordIndex));
            end
        else
            link_grad=cat(2,link_grad,0);
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
Val1=L(1)*P(1)+L(2)*P(2)+L(3);
Val2=norm([L(1),L(2)]);

if (JtIndex==1 && coordIndex==1)
    grad=-2*L(1)*(dist-Val1/Val2)/Val2;
elseif (JtIndex==1 && coordIndex==2)
    grad=-2*L(2)*(dist-Val1/Val2)/Val2;
elseif (JtIndex==2 && coordIndex==1)
    grad=2*(dist-Val1/Val2)*(L(1)*Val1/Val2^3-P(1)/Val2);
elseif (JtIndex==2 && coordIndex==2)
    grad=2*(dist-Val1/Val2)*(L(2)*Val1/Val2^3-P(2)/Val2);
elseif (JtIndex==2 && coordIndex==3)
    grad=-2*(dist-Val1/Val2)/Val2;
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

