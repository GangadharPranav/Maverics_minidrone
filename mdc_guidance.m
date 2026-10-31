function [command,diagnostics] = mdc_guidance(position,R,G,B)
%#codegen
% Camera-only guidance at 5 Hz. Stages: takeoff/follow/align/descend/landed/
% hold/search. Branch choice uses heading and bounded observed-route memory.
p=double(position(:)); dt=.2;
persistent stage previousP heading holdXY landingXY stable seen lost zCommand ...
    searchCenter searchTicks searchAngle junctionActive junctionClear ...
    startMarkerBlocked startMarkerXY velocity bias alignTicks descentTicks retries lowCommit ...
    probeTicks probeOrigin probeDirection departureReady endpointEvidence finishProbeActive ...
    partialSeen approachXY approachOrigin approachTicks markerMemoryXY markerMemoryAge markerMemorySeen ...
    incomingAxisPoint incomingAxisDirection incomingAxisValid incomingAxisAge cornerActive cornerXY cornerIn cornerOut cornerTicks fastFrames
if isempty(stage)
    stage=0; previousP=p(1:2); heading=[0;-1]; holdXY=p(1:2);
    landingXY=p(1:2); stable=0; seen=0; lost=0; zCommand=0;
    searchCenter=p(1:2); searchTicks=0; searchAngle=0;
    junctionActive=false; junctionClear=0;
    startMarkerBlocked=false; startMarkerXY=p(1:2);
    velocity=zeros(2,1); bias=zeros(2,1); alignTicks=0; descentTicks=0; retries=0; lowCommit=false;
    probeTicks=0; probeOrigin=p(1:2); probeDirection=[1;0]; departureReady=false;
    endpointEvidence=0; finishProbeActive=false;
    partialSeen=0; approachXY=p(1:2); approachOrigin=p(1:2); approachTicks=0;
    markerMemoryXY=p(1:2);markerMemoryAge=1000;markerMemorySeen=0;
    incomingAxisPoint=p(1:2);incomingAxisDirection=[1;0];incomingAxisValid=false;incomingAxisAge=1000;
    cornerActive=false;cornerXY=p(1:2);cornerIn=[1;0];cornerOut=[1;0];cornerTicks=0;fastFrames=0;
end
incomingAxisAge=min(1000,incomingAxisAge+1);
velocity=.65*velocity+.35*(p(1:2)-previousP)/dt;
speed=norm(velocity); previousP=p(1:2);
height=max(.12,-p(3)); prior=heading;
[anchor,target,marker,visible,markerValid,newHeading,quality,nav,branches,partialMarker,partialValid,lineAxes]= ...
    mdc_vision(R,G,B,height,prior);
center=[(size(R,2)+1)/2;(size(R,1)+1)/2];
scale=height/(120/(2*tan(19*pi/180)));
yaw=p(4); rotation=[cos(yaw) -sin(yaw);sin(yaw) cos(yaw)];
transform=rotation*[0 -1;1 0];
anchorWorld=p(1:2)+transform*(anchor-center)*scale;
targetWorld=p(1:2)+transform*(target-center)*scale; %#ok<NASGU>
markerWorld=p(1:2)+transform*(marker-center)*scale;
partialWorld=p(1:2)+transform*(partialMarker-center)*scale;
if startMarkerBlocked && norm(p(1:2)-startMarkerXY)>.30
    startMarkerBlocked=false;
end
% Retain only a stable camera-observed disk/arc centre for a short time.
% This memory guides a bounded terminal approach; it never confirms descent.
markerMemoryAge=min(1000,markerMemoryAge+1);
if height>.35 && ~startMarkerBlocked && (markerValid || partialValid)
    cue=partialWorld;if markerValid,cue=markerWorld;end
    if markerMemorySeen==0 || norm(cue-markerMemoryXY)<.06
        if markerMemorySeen==0,markerMemoryXY=cue;else,markerMemoryXY=.7*markerMemoryXY+.3*cue;end
        markerMemorySeen=min(8,markerMemorySeen+1);markerMemoryAge=0;
    elseif markerMemoryAge>25
        markerMemoryXY=cue;markerMemorySeen=1;markerMemoryAge=0;
    end
end
if (stage==1 || stage==5) && finishProbeActive && seen==0 && ~startMarkerBlocked && ...
        markerMemorySeen>=3 && markerMemoryAge<=50 && norm(markerMemoryXY-p(1:2))<=.50
    stage=8;approachXY=markerMemoryXY;approachOrigin=p(1:2);approachTicks=0;
    finishProbeActive=false;endpointEvidence=0;probeTicks=0;
end
if stage==0
    mdc_route_select(p(1:2),transform*heading,zeros(2,8),zeros(1,8),p(1:2),false,true);
    zCommand=max(-.8,zCommand-.30*dt);
    % Suppress only a disk actually underneath takeoff, not every finish
    % within a fixed radius of the start of a short or returning track.
    if markerValid && height>.35 && norm(markerWorld-startMarkerXY)<.14
        startMarkerBlocked=true;
    end
    if abs(p(3)+.8)<.06, stable=stable+1; else, stable=0; end
    if stable>=5
        stable=0;
        if visible
            heading=initialDirection(heading,branches); departureReady=true;
            stage=1;
        else
            stage=6; searchCenter=p(1:2); searchTicks=0; searchAngle=0;
        end
    end
elseif stage==6
    zCommand=-.8;
    searchTicks=searchTicks+1;
    if visible
        delta=anchorWorld-p(1:2);
        holdXY=p(1:2)+limited(delta,.08);
        if norm(delta)<.07, stable=stable+1; else, stable=0; end
        if stable>=3
            stage=1; stable=0; lost=0;
            if ~departureReady
                heading=initialDirection(heading,branches); departureReady=true;
            elseif nav(2)>-.94
                heading=newHeading;
            end
        end
    else
        stable=0;
        radius=min(1.2,.04+.0025*searchTicks);
        searchAngle=searchAngle+min(.40,.05/max(radius,.08));
        targetXY=searchCenter+radius*[cos(searchAngle);sin(searchAngle)];
        holdXY=p(1:2)+limited(targetXY-p(1:2),.10);
    end
    if searchTicks>=900, stage=7; holdXY=p(1:2); lost=2004; end
elseif stage==1 || stage==5
    zCommand=-.8;
    % A confirmed incoming-only endpoint owns its bounded gap approach even
    % when the last stripe leaves the image. Count every active camera tick.
    if finishProbeActive, probeTicks=probeTicks+1; end
    if visible
        lost=0; stage=1;
        if nav(1)>=3
            junctionActive=true; junctionClear=0;
        elseif junctionActive
            junctionClear=junctionClear+1;
            if junctionClear>=3, junctionActive=false; end
        end
        dirs=zeros(2,8); confidence=zeros(1,8); branchPoints=zeros(2,8);
        for k=1:8
            dirs(:,k)=transform*branches(3:4,k); confidence(k)=branches(5,k);
            branchPoints(:,k)=p(1:2)+transform*(branches(1:2,k)-center)*scale;
        end
        % Lane heading in world coordinates from tracked ego-lane orientation
        trackDirWorld=transform*heading;
        if norm(trackDirWorld)>0.1, trackDirWorld=trackDirWorld/norm(trackDirWorld); else, trackDirWorld=[1;0]; end
        lineDirWorld=trackDirWorld; lead=0.14;
        [selected,routeStatus,~,refineOldExit]=mdc_route_select(p(1:2),trackDirWorld, ...
            dirs,confidence,anchorWorld,nav(1)>=3,false,branchPoints);
        if refineOldExit
            [anchor,target,marker,visible,markerValid,newHeading,quality,nav,branches,partialMarker,partialValid,lineAxes]= ...
                mdc_vision(R,G,B,height,prior,true);
            anchorWorld=p(1:2)+transform*(anchor-center)*scale;
            markerWorld=p(1:2)+transform*(marker-center)*scale;
            partialWorld=p(1:2)+transform*(partialMarker-center)*scale;
            for k=1:8
                dirs(:,k)=transform*branches(3:4,k);confidence(k)=branches(5,k);
                branchPoints(:,k)=p(1:2)+transform*(branches(1:2,k)-center)*scale;
            end
            [selected,routeStatus,~]=mdc_route_select(p(1:2),trackDirWorld, ...
                dirs,confidence,anchorWorld,nav(1)>=3,false,branchPoints);
        end
        % Establish terminal topology before any marker can interrupt tracking.
        rawCount=0;incomingOnly=false;allIncoming=true;
        for k=1:8
            if confidence(k)>.20
                rawCount=rawCount+1;
                incomingOnly=dot(dirs(:,k),trackDirWorld)<=-.90;
                allIncoming=allIncoming && dot(dirs(:,k),trackDirWorld)<-.50;
            end
        end
        terminalView=(rawCount>0 && allIncoming) || rawCount==0;
        finishContext=finishProbeActive || (selected==0 && terminalView);
        % Require stable, separated circular-arc evidence at a route terminus.
        % Two observations guide approach only; they never authorize descent.
        if partialValid && finishContext && ~startMarkerBlocked && seen==0 && ...
                dot(partialWorld-anchorWorld,trackDirWorld)>.05 && ...
                abs(det([trackDirWorld partialWorld-anchorWorld]))<.12
            if partialSeen==0 || norm(partialWorld-approachXY)<.08
                if partialSeen==0,approachXY=partialWorld;else,approachXY=.6*approachXY+.4*partialWorld;end
                partialSeen=min(3,partialSeen+1);
            else
                partialSeen=1;approachXY=partialWorld;
            end
        else
            partialSeen=max(0,partialSeen-1);
        end
        % Landing acquisition when circular finish marker is valid
        if markerValid && ~startMarkerBlocked && (finishContext || seen>0)
            newHeading=heading;
            if seen==0 || norm(markerWorld-landingXY)<.18
                if seen==0, landingXY=markerWorld; else, landingXY=.6*landingXY+.4*markerWorld; end
                seen=min(6,seen+1);
            else
                seen=1; landingXY=markerWorld;
            end
            targetWorld=landingXY;
            if ~finishProbeActive, probeTicks=0; end
            endpointEvidence=0;
        elseif seen>0
            % Brake on the first disk and keep the candidate through brief
            % occlusion, rather than steering back onto the incoming stripe.
            newHeading=heading; targetWorld=landingXY;
            seen=max(0,seen-.5);
        elseif partialSeen>=2
            stage=8;approachOrigin=p(1:2);approachTicks=0;
            targetWorld=approachXY;newHeading=heading;
            finishProbeActive=false;endpointEvidence=0;probeTicks=0;
        elseif selected>0
            newHeading=branches(3:4,selected);
            lineDirWorld=transform*newHeading;
            if lineAxes(5,selected)>=.42
                axisPointWorld=p(1:2)+transform*(lineAxes(1:2,selected)-center)*scale;
                axisDirectionWorld=transform*lineAxes(3:4,selected);
                if dot(axisDirectionWorld,lineDirWorld)>.85
                    % The fitted axis is used for corner geometry; ordinary tracking is preserved.
                    if ~cornerActive && incomingAxisValid && incomingAxisAge<=20 && ...
                            dot(incomingAxisDirection,trackDirWorld)>.94 && ...
                            abs(det([incomingAxisDirection p(1:2)-incomingAxisPoint]))<.04 && ...
                            dot(incomingAxisDirection,axisDirectionWorld)<cos(.60)
                        denominator=det([incomingAxisDirection axisDirectionWorld]);
                        if abs(denominator)>.15
                            along=det([axisPointWorld-incomingAxisPoint axisDirectionWorld])/denominator;
                            vertex=incomingAxisPoint+along*incomingAxisDirection;
                            forward=dot(vertex-p(1:2),incomingAxisDirection);
                            pixel=round(center+transform'*(vertex-p(1:2))/scale);
                            onPaint=false;
                            if pixel(1)>=2&&pixel(1)<=size(R,2)-1&&pixel(2)>=2&&pixel(2)<=size(R,1)-1
                                red=double(R(pixel(2),pixel(1)));green=double(G(pixel(2),pixel(1)));blue=double(B(pixel(2),pixel(1)));
                                onPaint=red>45&&red>1.45*green&&red>1.45*blue&&red-max(green,blue)>28;
                            end
                            if forward>=-.025&&forward<=.23&&norm(vertex-p(1:2))<=.25&&onPaint
                                cornerActive=true;cornerXY=vertex;cornerIn=incomingAxisDirection;
                                cornerOut=axisDirectionWorld;cornerTicks=0;
                            end
                        end
                    elseif ~cornerActive && dot(trackDirWorld,axisDirectionWorld)>.94
                        incomingAxisPoint=axisPointWorld;incomingAxisDirection=axisDirectionWorld;incomingAxisValid=true;incomingAxisAge=0;
                    end
                end
            end
            turnEst=acos(max(-1,min(1,dot(heading,newHeading))));
            if ~cornerActive && turnEst>.60,incomingAxisValid=false;end
            % Short lookahead (6-10 cm); tracking accuracy still requires flight validation.
            lead=max(.04,min(.10,.10/(1.0+1.5*turnEst)));
            % Increase progress only after three clean, centred straight observations.
            % Seven isolated cross-sections verify paint through 19 cm of preview.
            trackOffset=abs(det([lineDirWorld anchorWorld-p(1:2)]));
            if nav(1)==1 && lineAxes(5,selected)>=.99 && ...
                    dot(heading,newHeading)>.995 && trackOffset<.015 && nav(3)>.22 && ~cornerActive
                fastFrames=min(3,fastFrames+1);
            else
                fastFrames=0;
            end
            if fastFrames>=3,lead=.16;end
            targetWorld=anchorWorld+lead*lineDirWorld;
            seen=max(0,seen-1);
            probeTicks=0; endpointEvidence=0; finishProbeActive=false;
        else
            % The app leaves a gap between the line end and finish circle.
            % Cross it along the observed heading, with a strict travel/time
            % bound. Pulling back to the incoming anchor deadlocked here.
            newHeading=heading;
            seen=max(0,seen-.5);
            if probeTicks==0
                probeOrigin=anchorWorld; probeDirection=trackDirWorld;
            end
            % Do not infer a finish from an arbitrary loss or from branches
            % rejected by history. Require two near-nadir observations of
            % exactly one incoming branch, with no observed outgoing exit.
            rawCount=0; incomingOnly=false;
            for k=1:8
                if confidence(k)>.20
                    rawCount=rawCount+1;
                    incomingOnly=dot(dirs(:,k),trackDirWorld)<=-.90;
                end
            end
            credibleEndpoint=rawCount>0 && allIncoming && ...
                norm(anchorWorld-p(1:2))<=.12;
            if credibleEndpoint
                endpointEvidence=min(2,endpointEvidence+1);
            elseif ~finishProbeActive
                endpointEvidence=0;
            end
            wasActive=finishProbeActive;
            if endpointEvidence>=2, finishProbeActive=true; end
            if ~wasActive, probeTicks=probeTicks+1; end
            if norm(p(1:2)-probeOrigin)<.50 && probeTicks<=75
                % Keep the last observed stripe corridor while crossing the gap.
                % Pure heading extrapolation from an off-centre vehicle can
                % leave only a sector of the pad visible at the image edge.
                along=dot(p(1:2)-probeOrigin,probeDirection);
                targetWorld=probeOrigin+(along+.08)*probeDirection;
            else
                targetWorld=p(1:2);
                if finishProbeActive
                    % Keep the terminal corridor; an expanding search can
                    % recapture the incoming stripe and retrace the route.
                    stage=7; lost=2005;
                else
                    stage=6; searchCenter=p(1:2); searchTicks=0; searchAngle=0;
                end
                stable=0; probeTicks=0; endpointEvidence=0; finishProbeActive=false;
            end
        end
        turn=max(-1,min(1,dot(heading,newHeading)));
        turnAngle=acos(turn);
        heading=newHeading;
        zCommand=-.8;
        % Reference lead cap in metres; actual speed is set by the position controller.
        vCmd=max(.025,min(.08,.08/(1+2.5*turnAngle)));
        if fastFrames>=3 && seen==0 && selected>0 && ~cornerActive,vCmd=.12;end
        if turnAngle>1.2, vCmd=min(vCmd,.03); end
        if seen==0 && selected>0 && speed>.03
            travelDirection=transform*newHeading;
            if dot(velocity,travelDirection)/speed<.70, vCmd=min(vCmd,.035); end
        end
        if selected==0, vCmd=min(vCmd,.04); end
        trackError=targetWorld-p(1:2);
        holdXY=p(1:2)+limited(trackError,vCmd);
        if routeStatus>=2, stage=7; holdXY=p(1:2); lost=2000+routeStatus-1; end
        if seen>=4
            stage=2; stable=0; holdXY=landingXY; alignTicks=0; retries=0; bias(:)=0;
        end
    else
        lost=lost+1; stage=5; seen=max(0,seen-.5);
        if finishProbeActive
            if norm(p(1:2)-probeOrigin)<.50 && probeTicks<=75
                along=dot(p(1:2)-probeOrigin,probeDirection);
                targetXY=probeOrigin+(along+.08)*probeDirection;
                holdXY=p(1:2)+limited(targetXY-p(1:2),.05);
            else
                stage=7; holdXY=p(1:2); lost=2005;
                probeTicks=0; endpointEvidence=0; finishProbeActive=false;
            end
        else
            % Unexpected mid-track image loss is not endpoint evidence.
            if lost==1, holdXY=p(1:2); end
            endpointEvidence=0;
            if lost==15
                stage=6; searchCenter=p(1:2); searchTicks=0; searchAngle=0;
                junctionActive=false; stable=0;
            end
        end
    end
elseif stage==8
    % Camera-only bounded marker acquisition survives loss of the last line.
    % Keep altitude until FOUR strict full-circle confirmations, as in follow.
    zCommand=-.8;approachTicks=approachTicks+1;lost=0;
    if markerValid && norm(markerWorld-approachXY)<.12
        approachXY=.6*approachXY+.4*markerWorld;
        landingXY=approachXY;seen=min(6,seen+1);
    else
        seen=max(0,seen-.5);
        if partialValid && norm(partialWorld-approachXY)<.10
            approachXY=.7*approachXY+.3*partialWorld;
        end
    end
    holdXY=p(1:2)+limited(approachXY-p(1:2),.05);
    if seen>=4
        stage=2;stable=0;holdXY=landingXY;alignTicks=0;retries=0;bias(:)=0;
    elseif approachTicks>=75 || norm(p(1:2)-approachOrigin)>.50
        stage=7;holdXY=p(1:2);lost=2006;seen=0;
    end
elseif stage==2
    zCommand=-.45; alignTicks=alignTicks+1;
    if markerValid && norm(markerWorld-landingXY)<.12, landingXY=.7*landingXY+.3*markerWorld; end
    bias=limited(bias+.25*(landingXY-p(1:2))*dt,.10);
    holdXY=landingXY+bias;
    if norm(p(1:2)-landingXY)<.060 && speed<.10 && p(3)<-.32
        stable=stable+1;
    else
        stable=0;
    end
    if stable>=3
        stage=3; zCommand=p(3); descentTicks=0; lowCommit=false;
    elseif alignTicks>=100
        stage=7; holdXY=p(1:2); lost=2003;
    end
elseif stage==3
    descentTicks=descentTicks+1;
    if markerValid && height>.25 && norm(markerWorld-landingXY)<.12
        landingXY=.85*landingXY+.15*markerWorld;
    end
    error=landingXY-p(1:2); lateral=norm(error);
    if ~lowCommit && -p(3)>.25, bias=limited(bias+.25*error*dt,.10); end
    holdXY=landingXY+bias;
    if -p(3)<=.35 && lateral<.075 && speed<.12, lowCommit=true; end
    % The supplied controller suppresses XY correction below 25 cm.
    % Once aligned above it, finish gently within the disk drift bound.
    if (lowCommit && lateral>.12) || (~lowCommit && descentTicks>80)
        retries=retries+1; stage=2; zCommand=-.45; alignTicks=0; stable=0; lowCommit=false;
        if retries>2, stage=7; lost=2003; holdXY=p(1:2); end
    elseif lowCommit || (lateral<.075 && speed<.14)
        zCommand=min(.04,zCommand+.075*dt);
    end
    if stage==3 && p(3)>-.025 && zCommand>=0, stage=4; end
elseif stage==4
    holdXY=landingXY+bias; zCommand=.04;
elseif stage==7
    % Bounded hover for a loop without an exit/marker or failed alignment.
    % Never invent a route or land on an unmarked patch.
    if markerValid && ~startMarkerBlocked && (lost==2005 || lost==2006)
        seen=seen+1;
        if seen>=4
            landingXY=markerWorld; stage=2; alignTicks=0; stable=0; retries=0; bias(:)=0;
        end
    else
        seen=0;
    end
end
if cornerActive && (stage==1 || stage==5)
    % Reach the observed centreline intersection before departing on the exit.
    % This remains follow stage, so every apex sample is included in scoring.
    cornerTicks=cornerTicks+1;holdXY=p(1:2)+limited(cornerXY-p(1:2),.04);
    heading=transform'*cornerIn;
    if (norm(cornerXY-p(1:2))<.020&&speed<.04)||cornerTicks>=60
        cornerActive=false;heading=transform'*cornerOut;
        incomingAxisPoint=cornerXY;incomingAxisDirection=cornerOut;incomingAxisValid=true;incomingAxisAge=0;
    end
elseif stage~=1 && stage~=5
    cornerActive=false;
end
command=single([holdXY;zCommand;0]);
diagnostics=[stage;double(visible);double(markerValid);quality; ...
    anchor;target;marker;heading;speed;lost;zCommand];
end
function delta=limited(delta,limit)
length=norm(delta);
if length>limit, delta=delta*(limit/length); end
end

function direction=initialDirection(previous,branches)
direction=previous;candidate=previous;count=0;
for k=1:8
    if branches(5,k)>.20,count=count+1;candidate=branches(3:4,k);end
end
if count==1,direction=candidate;end
end
