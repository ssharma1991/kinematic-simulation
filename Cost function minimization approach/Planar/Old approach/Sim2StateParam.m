function [stateInfo]= Sim2StateParam (initData)
Joints=initData.Joints;
Links=initData.Links;
nJts=length(Joints);
nLinks=length(Links);

% Find pivot which acts as input (INPUT PIVOT)
% Assumption- Single input joint ONLY
input_joint_index=0;
for i=1:nJts
    if isequal(Joints{i}{end},'input')
        input_joint_index=i;
    end
end
inputInfo={input_joint_index, Joints{input_joint_index}{1:end}};

% Find pivots which will move directly by input(PERTURBED PIVOTS)
% Assumption- Single input link ONLY
perturbed_joints_index={};
for i=1:nLinks
    if ~isequal(Links{i}{end},'G')
        jt_index = find([Links{i}{:}] == input_joint_index);
        if ~isempty(jt_index)
            perturbed_joints_index=Links{i}{1};
            perturbed_joints_index(jt_index)=[];
        end
    end
end
perturbedInfo={};
for i=1:nJts
    if ~isempty(find(perturbed_joints_index == i))
        perturbedInfo{end+1}={i, Joints{i}{:}};
    end
end
% Find pivots which remain fixed (FIXED PIVOTS)
fixed_joints_index={};
for i=1:nLinks
    if isequal(Links{i}{end},'G')
        fixed_joints_index=Links{i}{1};
        fixed_joints_index(find([Links{i}{1}]==input_joint_index))=[];
    end
end
fixedInfo={};
for i=1:nJts
    if ~isempty(find(fixed_joints_index == i))
        fixedInfo{end+1}={i, Joints{i}{1:end}};
    end
end
% Generate the state (All joints - Fixed/Input/Perturbed joints)
x_0=[];
x_add={};
for i=1:nJts
    isInput=(i==input_joint_index);
    isFixed=~isempty(find(fixed_joints_index == i));
    isPerturbed=~isempty(find(perturbed_joints_index == i));
    isSpecial=isInput||isFixed||isPerturbed;
    
    if ~isSpecial
        if (isequal(Joints{i}{1},'P'))
            x_0(end+1:end+3)=Joints{i}{2};
            x_add{end+1}={i,'P'};
        else
            x_0(end+1:end+2)=Joints{i}{2};
            x_add{end+1}={i,'R'};
        end
    end
end

stateInfo.state=x_0;
stateInfo.stateIndex=x_add;
stateInfo.input=inputInfo;
stateInfo.perturbed=perturbedInfo;
stateInfo.fixed=fixedInfo;
stateInfo.Links=initData.Links;
end