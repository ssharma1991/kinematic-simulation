clc
clear all
close all

%INPUTS
specData=Inp_RRRR();
%specData=Inp_RRRP();
%specData=Inp_RRPR();
%specData=Inp_RRPP();
%specData=Inp_janson();
specData=Inp_jansonP();
%specData=Inp_stephenson2;
%specData=Inp_doubleButterfly();
%specData=Inp_10bar();

[Cplr,inp_ang]=Simulate(specData);

function [CplrPath,ang]= Simulate (inpSpecData)
global newState initMechConstants
ang=[];

init_SpecPlotData=initSpecPlotData(inpSpecData);
PlotLinkage(init_SpecPlotData);
initSimData=Spec2SimParam(inpSpecData);
initMechConstants=SimParam2MechConstants(initSimData);
stateData=Sim2StateParam(initSimData);
x0=stateData.state;
plotData={};
incrInterval=2*pi/90;
tolerance=1e-3;
incompleteLoop=false;
tic
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

if (incompleteLoop)
    plotData2={};
    x0=stateData.state;
    ang=flipud(ang);
    for incr=-incrInterval:-incrInterval:-2*pi
        ang=cat(1,ang,incr);
        newState=StatePerturb(stateData, incr);
        
        %options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
        %options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true,'FunctionTolerance',1e-10,'StepTolerance',1e-10);
        [x,fval,exitflag,output] = fsolve(@Residual,x0);
        x0=x;
        newState.state=x;
        newSimData=State2SimParam(newState);
        
        if (fval<tolerance)
            plotData2{end+1}=SpecPlotData(initSimData,newSimData,init_SpecPlotData);
        else
            break
        end
    end
end
toc
% Plot the path for Theo Jansen leg
% cplrLinkIndex=7;cplrPtIndex=3;
% Plot the path for Stephenson
% cplrLinkIndex=4;cplrPtIndex=2;
cplrLinkIndex=1;cplrPtIndex=2;
leg=[];
for i=1:length(plotData)
    leg=cat(1,leg,plotData{i}.Links{cplrLinkIndex}(cplrPtIndex,:));
end
plot(leg(:,1),leg(:,2));
CplrPath=leg;% Plot the path for Theo Jansen leg


if (~incompleteLoop)
    for i=1:length(plotData)
        cla;
        PlotLinkage(plotData{i});
        axis equal
        if i==1
            gif('closeLoop.gif');
        else
            gif
        end
    end
else
    newPlotData=[plotData, fliplr(plotData), plotData2, fliplr(plotData2)];
    for i=1:length(newPlotData)
        cla;
        PlotLinkage(newPlotData{i});
        if i==1
            gif('openLoop.gif');
        else
            gif
        end
    end
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
