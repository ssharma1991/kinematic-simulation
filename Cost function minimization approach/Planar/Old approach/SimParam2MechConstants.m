function [MechConst]= SimParam2MechConstants(Data)
% CALCULATE MECHANISM CONSTANTS- length and angle
MechConst={};
for i=1:length(Data.Links)
    MechConst{i}=Data2LinkConstraints(Data,i);
end
end
function [LinkConst]= Data2LinkConstraints(Data,LinkIndex)
Jts=Data.Links{LinkIndex}{1};
nJts=length(Jts);
LinkConst=zeros(nJts);
for i=1:nJts-1
    jt_first=Data.Joints{Jts(i)};
    for j=i+1:nJts
        jt_second=Data.Joints{Jts(j)};
        if (isequal(jt_first{1},'R') && isequal(jt_second{1},'R'))
            LinkConst(i,j)=RRconst(jt_first{2},jt_second{2});
        elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'P'))
            LinkConst(i,j)=PPconst(jt_first{2},jt_second{2});
        elseif (isequal(jt_first{1},'R') && isequal(jt_second{1},'P'))
            LinkConst(i,j)=RPconst(jt_first{2},jt_second{2});
        elseif (isequal(jt_first{1},'P') && isequal(jt_second{1},'R'))
            LinkConst(i,j)=RPconst(jt_second{2},jt_first{2});
        end
    end
end
end
function [dist]= RRconst(P1,P2)
dist=norm(P1-P2);
end
function [dist]= RPconst(P,L)
dist=abs(L(1)*P(1)+L(2)*P(2)+L(3))/norm([L(1),L(2)]);
end
function [cosAng]= PPconst(L1,L2)
cosAng=(L1(1)*L2(1)+L1(2)*L2(2))/(norm([L1(1),L1(2)])*norm([L2(1),L2(2)]));
end
