function mdc_prepare_simulation
% Prepare simulator placement; no scene geometry enters autonomous guidance.
% Model PreLoad callbacks run before final start placement and wind restore.
hadWind=evalin('base','exist(''windVel'',''var'')==1');
wind=[];
if hadWind
    wind=evalin('base','windVel');
    validateattributes(wind,{'numeric'},{'vector','numel',3,'finite','real'});
    wind=double(wind(:));
end
restoreWind=onCleanup(@()restoreRequestedWind(hadWind,wind));
try
    project=currentProject;
catch
    openProject(fullfile(pwd,'PX4MultiCopterPerceptionAndControl.prj'));
    project=currentProject;
end
assignin('base','Simulation',true);
assignin('base','MainCamera1','MainCamera1');
root=char(project.RootFolder);
model='PX4MultiCopterPerceptionAndControl';
% These are the two actual ModelNameDialog references in the starter main.
names={model,'PerceptionAndGuidance','flightControlSystem'};
folders={'mainModels','perception','controller'};
for k=1:numel(names)
    expected=fullfile(root,folders{k},[names{k} '.slx']);
    if ~bdIsLoaded(names{k})
        load_system(expected);
    else
        assert(strcmpi(strrep(get_param(names{k},'FileName'),'\','/'), ...
            strrep(expected,'\','/')), ...
            'Mavericks:ForeignLoadedModel', ...
            'Loaded model %s belongs to another project. Close that model first.',names{k});
    end
end
ready=evalin('base',[ ...
    'exist(''MPC_THR_HOVER'',''var'') && exist(''SampleTime'',''var'') && ' ...
    'exist(''init'',''var'') && isstruct(init) && isfield(init,''equilibriumZ'') && ' ...
    'exist(''UAVStatesBus'',''var'') && exist(''desiredSetpoint'',''var'')']);
if ~ready
    evalin('base','InitVars; setupParameters;');
end
% All startup/PreLoad resets have now completed, including new references.
restoreWind=[]; %#ok<NASGU>
fixture=fullfile(root,'initData.mat');
if isfile(fixture)
    data=load(fixture);
    assert(isfield(data,'Lines') && ~isempty(data.Lines),'The track needs at least one line.');
    assert(isfield(data,'Circle') && ~isempty(data.Circle),'Place the landing circle in Track Builder.');
    initial=evalin('base','init');
    initial.posNED=[data.Lines(1).StartX-.5,data.Lines(1).StartY-1,initial.equilibriumZ];
    if evalin('base','exist(''mdcStartOffset'',''var'')')
        offset=evalin('base','mdcStartOffset');
        validateattributes(offset,{'double'},{'vector','numel',2,'finite','real'});
        initial.posNED(1:2)=initial.posNED(1:2)+reshape(offset,1,2);
    end
    assignin('base','init',initial);
end
clear mdc_guidance mdc_vision mdc_route_select;
repairBundledScenePath(model);
end

function restoreRequestedWind(hadWind,wind)
if hadWind,assignin('base','windVel',wind);end
end

function repairBundledScenePath(model)
% A saved Windows installation path must not bind a portable project to E:.
% Only the known bundled Default Scenes executable is eligible for repair.
% Explicit custom Unreal executables and non-Windows setups remain untouched.
if ~ispc, return; end
blocks=find_system(model,'LookUnderMasks','all','FollowLinks','on', ...
    'MatchFilter',@Simulink.match.allVariants, ...
    'Name','Simulation 3D Scene Configuration');
tail='toolbox/shared/sim3d_projects/automotive_project_windows/UE/WindowsNoEditor/VehicleSimulation.exe';
local=fullfile(matlabroot,'toolbox','shared','sim3d_projects', ...
    'automotive_project_windows','UE','WindowsNoEditor','VehicleSimulation.exe');
for k=1:numel(blocks)
    format=get_param(blocks{k},'ProjectFormat');
    saved=get_param(blocks{k},'ProjectName');
    normalized=strrep(char(saved),'\','/');
    if strcmp(format,'Default Scenes') && endsWith(normalized,tail,'IgnoreCase',true)
        if ~isfile(local)
            error('Mavericks:MissingSimulation3DProject', ...
                ['The bundled Simulation 3D scene executable is missing: %s. ' ...
                 'Install the competition-required Simulation 3D support before running.'],local);
        end
        if ~strcmpi(normalized,strrep(local,'\','/'))
            set_param(blocks{k},'ProjectName',local);
        end
    end
end
end
