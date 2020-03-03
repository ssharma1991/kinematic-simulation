clc
clear all
close all

%INPUTS
specData=Inp_RRRR();
specData=Inp_RRRP();
specData=Inp_RRPR();
specData=Inp_RRPP();
%specData=Inp_janson();
specData=Inp_jansonP();
%specData=Inp_doubleButterfly();
%specData=Inp_10bar();

Simulate(specData);

function []= Simulate (inpSpecData)
global newState initMechConstants

init_SpecPlotData=initSpecPlotData(inpSpecData);
PlotLinkage(init_SpecPlotData);
initSimData=Spec2SimParam(inpSpecData);
initMechConstants=SimParam2MechConstants(initSimData);
stateData=Sim2StateParam(initSimData);
x0=stateData.state;
plotData={};
incrInterval=pi/45;
tolerance=1e-6;
incompleteLoop=false;
for incr=0:incrInterval:2*pi
    newState=StatePerturb(stateData, incr);
    
    %options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
    options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true,'FunctionTolerance',1e-10,'StepTolerance',1e-10);
    [x,fval,exitflag,output] = fminunc(@Cost,x0,options)
    x0=x;
    newState.state=x;
    newSimData=State2SimParam(newState);    
    
    if (fval<tolerance)
        plotData{end+1}=SpecPlotData(initSimData,newSimData,init_SpecPlotData);
    else
        incompleteLoop=true;
        break
    end
end

if (incompleteLoop)
    plotData2={};
    x0=stateData.state;
    for incr=0:-incrInterval:-2*pi
        newState=StatePerturb(stateData, incr);
        
        %options = optimoptions(@fminunc,'Algorithm','quasi-newton','Display','iter');
        options = optimoptions('fminunc','Algorithm','trust-region','SpecifyObjectiveGradient',true,'FunctionTolerance',1e-10,'StepTolerance',1e-10);
        [x,fval,exitflag,output] = fminunc(@Cost,x0,options)
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

pauseTime=.1;
if (~incompleteLoop)
    for i=1:length(plotData)
        cla;
        PlotLinkage(plotData{i});
        pause(pauseTime)
    end
else
    for i=1:length(plotData)
        %cla;
        PlotLinkage(plotData{i});
        pause(pauseTime)
    end
    for i=length(plotData):-1:1
        %cla;
        PlotLinkage(plotData{i});
        pause(pauseTime)
    end
    for i=1:length(plotData2)
        %cla;
        PlotLinkage(plotData2{i});
        pause(pauseTime)
    end
    for i=length(plotData2):-1:1
        %cla;
        PlotLinkage(plotData2{i});
        pause(pauseTime)
    end
    
end
end
function [err,grad]= Cost(x)
global newState initMechConstants

newState.state=x;
newSimData=State2SimParam(newState);
newMechConstants= SimParam2MechConstants(newSimData);

err=0;
for i=1:length(initMechConstants)
    %initMechConstants{i}
    %newMechConstants{i}
    error=norm(initMechConstants{i}-newMechConstants{i})^2;
    err=err+error;
end

if nargout > 1 % gradient required
    grad=Gradient(newSimData, initMechConstants, newState.stateIndex);
end
end


%ToDO
function [H]= Hessian(x)
end