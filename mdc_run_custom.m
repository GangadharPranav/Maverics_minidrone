function output=mdc_run_custom(stopTime,recordCamera,show3D)
% Prepare and run the current Track Builder scene with useful flight logs.
% Return a Simulink.SimulationOutput, and write results into work/custom_run.
if nargin<1, stopTime=360; end
if nargin<2, recordCamera=false; end
if nargin<3, show3D=true; end
validateattributes(recordCamera,{'logical'},{'scalar'});
validateattributes(show3D,{'logical'},{'scalar'});
validateattributes(stopTime,{'numeric'},{'scalar','finite','positive'});
mdc_prepare_simulation;
mdl='PX4MultiCopterPerceptionAndControl';
scene=find_system(mdl,'LookUnderMasks','all','FollowLinks','on', ...
    'Name','Simulation 3D Scene Configuration');
if ~isempty(scene)
    previousWindow=get_param(scene{1},'EnableWindow');
    restoreWindow=onCleanup(@() set_param(scene{1},'EnableWindow',previousWindow)); %#ok<NASGU>
    if show3D, set_param(scene{1},'EnableWindow','on');
    else, set_param(scene{1},'EnableWindow','off'); end
end
set_param(mdl,'SignalLogging','on','SignalLoggingName','logsout');
blockNames={'Visualization Subsystem','UAV Dynamics','PerceptionGuidance'};
signalNames={'camera','states','setpoint'};
for k=1:numel(blockNames)
    ports=get_param([mdl '/' blockNames{k}],'PortHandles');
    if k==1 && ~recordCamera
        set_param(ports.Outport(1),'DataLogging','off');
        continue;
    end
    set_param(ports.Outport(1),'DataLogging','on', ...
        'DataLoggingNameMode','Custom','DataLoggingName',signalNames{k});
    if k==2
        set_param(ports.Outport(1),'DataLoggingDecimateData','on', ...
            'DataLoggingDecimation','100');
    end
end
output=sim(mdl,'StopTime',num2str(stopTime),'ReturnWorkspaceOutputs','on');
try
    project=currentProject;
catch
    openProject(fullfile(pwd,'PX4MultiCopterPerceptionAndControl.prj'));
    project=currentProject;
end
folder=fullfile(project.RootFolder,'work','custom_run');
if ~isfolder(folder), mkdir(folder); end
s=output.logsout.get('states').Values;
xyz=[s.X.Data(:),s.Y.Data(:),s.Z.Data(:)];
writematrix([s.X.Time,xyz],fullfile(folder,'trajectory.csv'));
save(fullfile(folder,'flight.mat'),'output','-v7.3');
figure('Name','Custom track flight');
plot(xyz(:,1)+.5,xyz(:,2)+1,'b-','LineWidth',1.5);
hold on;
fixture=fullfile(project.RootFolder,'initData.mat');
if isfile(fixture)
    scene=load(fixture);
    for k=1:numel(scene.Lines)
        line=scene.Lines(k);
        plot([line.StartX line.EndX],[line.StartY line.EndY],'r--');
    end
    plot(scene.Circle.X,scene.Circle.Y,'ro','MarkerSize',12);
    errorM=norm(xyz(end,1:2)+[.5 1]-[scene.Circle.X scene.Circle.Y]);
    fprintf('Final target-center error: %.3f m. Final altitude: %.3f m.\n',errorM,-xyz(end,3));
end
axis equal; grid on; xlabel('Arena X (m)'); ylabel('Arena Y (m)');
title('Blue: flown path. Red: custom track.');
exportgraphics(gcf,fullfile(folder,'trajectory.png'));
fprintf('Flight data saved to %s\n',folder);
end
