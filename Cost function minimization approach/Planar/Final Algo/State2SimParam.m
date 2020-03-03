function [Data]= State2SimParam (stateInfo)
x_0= stateInfo.state;
x_add=stateInfo.stateIndex;
inputInfo=stateInfo.input;
perturbedInfo=stateInfo.perturbed;
fixedInfo=stateInfo.fixed;

% Get all Input/Perturbed/Fixed joints data
Joints{inputInfo{1}}=inputInfo(2:end);
for i=1:length(perturbedInfo)
    Joints{perturbedInfo{i}{1}}=perturbedInfo{i}(2:end);
end
for i=1:length(fixedInfo)
    Joints{fixedInfo{i}{1}}=fixedInfo{i}(2:end);
end

% Get all state joints data
x0_index=1;
for i=1:length(x_add)
    if (isequal(x_add{i}{2},'R'))
        Joints{x_add{i}{1}}={x_add{i}{2}, x_0(x0_index:x0_index+1)};
        x0_index=x0_index+2;
    elseif (isequal(x_add{i}{2},'P'))
        Joints{x_add{i}{1}}={x_add{i}{2}, x_0(x0_index:x0_index+2)};
        x0_index=x0_index+3;
    end
end

Data.Joints=Joints;
Data.Links=stateInfo.Links;
end