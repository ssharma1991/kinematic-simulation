clc
clear all
close all

%INPUTS
specData=Inp_10bar();
%specData=Inp_stephenson2();

[Cplr,inp_ang]=Simulate(specData);

function [CplrPath,ang]= Simulate (inpSpecData)
global newState initMechConstants
ang=[];

init_SpecPlotData=initSpecPlotData(inpSpecData);
PlotLinkage(init_SpecPlotData,[0,0]);
initSimData=Spec2SimParam(inpSpecData);
initMechConstants=SimParam2MechConstants(initSimData);
stateData=Sim2StateParam(initSimData);
x0=stateData.state;
plotData={};
incrInterval=2*pi/60;
tolerance=1e-3;
incompleteLoop=false;

for incr=0:incrInterval:2*pi
    ang=cat(1,ang,incr);
    newState=StatePerturb(stateData, incr);
    
    %options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    options = optimoptions('fsolve','Algorithm','trust-region','SpecifyObjectiveGradient',true,'FunctionTolerance',1e-3,'StepTolerance',1e-3);
    [x,fval,exitflag,output] = fsolve(@Residual,x0);
    x0=x;
    newState.state=x;
    newSimData=State2SimParam(newState);    
    
    if (norm(fval)<tolerance)
        plotData{end+1}=SpecPlotData(initSimData,newSimData,init_SpecPlotData);
    else
        incompleteLoop=true;
        break
    end
end


cplrLinkIndex=9;cplrPtIndex=1;
leg=[];
for i=1:length(plotData)
    leg=cat(1,leg,plotData{i}.Links{cplrLinkIndex}(cplrPtIndex,:));
end
CplrPath=leg;% Plot the path for Theo Jansen leg


if (~incompleteLoop)
    for i=1:length(plotData)
        cla;
        PlotLinkage(plotData{i},CplrPath);
        axis equal
        xlim([0 7]) 
        ylim([-3 4])
        if i==1
            gif('closeLoop.gif');
        else
            gif
        end
    end
else
end

end
function [err,Jac]= Residual(x)
global newState initMechConstants

newState.state=x;
newSimData=State2SimParam(newState);
newMechConstants= SimParam2MechConstants(newSimData);

err=[];
for i=1:length(initMechConstants)
    %initMechConstants{i}
    %newMechConstants{i}
    isFixedLink=(length(newSimData.Links{i})==2);
    isInputLink=(i==newState.inpLinks{1});
    if ~(isFixedLink || isInputLink)
        error=(initMechConstants{i}-newMechConstants{i}).^2;
        err=cat(1,err,error);
    end
end

if nargout > 1 % gradient required
    Jac=Gradient(newSimData, initMechConstants, newState.stateIndex,newState.inpLinks);
end

end
