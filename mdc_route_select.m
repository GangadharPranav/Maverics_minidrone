function [selected,routeStatus,lockedDirection,refineOldExit] = mdc_route_select( ...
    position,heading,branchDirections,confidence,observedAnchor,~,reset,branchPoints)
%#codegen
% Heading continuity with bounded online route history for line following.
% 1. Prefer unvisited exits; otherwise prefer continuous visible paint.
% 2. At corners/bends: prioritize the outgoing turn and heavily penalize 180-deg reversal.
% 3. At line terminus: detect that no forward/lateral branch exists and signal End of Track.

if nargin<8, useHistory=false;branchPoints=zeros(2,8);else,useHistory=true;end
persistent previousP distance sectionCount lastTurnAngle trailXY trailS trailIndex trailCount trailDirections ...
    cycleXY cycleDirs cycleS cycleIndex cycleCount cycleMatches cycleActive cycleLength cycleStartS cycleUnseen

if isempty(distance) || reset
    previousP=position; distance=0;
    sectionCount=1; lastTurnAngle=0;
    trailXY=zeros(2,384);trailS=zeros(1,384);trailIndex=1;trailCount=1;
    trailXY(:,1)=observedAnchor;trailDirections=zeros(2,384);trailDirections(:,1)=heading;
    cycleXY=zeros(2,256);cycleDirs=zeros(2,256);cycleS=zeros(1,256);
    cycleIndex=1;cycleCount=1;cycleMatches=0;cycleActive=false;
    cycleLength=0;cycleStartS=0;cycleUnseen=0;
    cycleXY(:,1)=observedAnchor;cycleDirs(:,1)=heading;
end

selected=0; routeStatus=0; lockedDirection=heading; refineOldExit=false;
historyDistance=ones(1,8)*1e6;
if reset, return; end

% Update path distance
step=norm(position-previousP);
distance=distance+min(.30,step);
previousP=position;
% Bounded online cycle memory. Observations only: no supplied map or goal.
% Matching direction excludes ordinary opposite-direction hairpin returns.
matchedLength=0;
if step>.001
    for k=1:256
        if k<=cycleCount && distance-cycleS(k)>.8 && ...
                norm(observedAnchor-cycleXY(:,k))<.12 && dot(heading,cycleDirs(:,k))>.90
            matchedLength=max(matchedLength,distance-cycleS(k));
        end
    end
    if matchedLength>0,cycleMatches=cycleMatches+1;else,cycleMatches=0;end
    if ~cycleActive && cycleMatches>=3
        cycleActive=true;cycleLength=matchedLength;cycleStartS=distance;cycleUnseen=0;
    end
    if cycleActive
        if matchedLength==0,cycleUnseen=cycleUnseen+min(.30,step);else,cycleUnseen=0;end
        if cycleUnseen>.35,cycleActive=false;cycleMatches=0;end
    end
    if distance-cycleS(cycleIndex)>=.20
        cycleIndex=mod(cycleIndex,256)+1;cycleCount=min(256,cycleCount+1);
        cycleXY(:,cycleIndex)=observedAnchor;cycleDirs(:,cycleIndex)=heading;cycleS(cycleIndex)=distance;
    end
end

if distance-trailS(trailIndex)>=.05
    trailIndex=mod(trailIndex,384)+1;trailCount=min(384,trailCount+1);
    trailXY(:,trailIndex)=observedAnchor;trailS(trailIndex)=distance;trailDirections(:,trailIndex)=heading;
end
% Camera border chords can point sideways toward an older leg rather than
% directly backwards. Reject an exit lying on the recently travelled trail.
% Store observed stripe centres rather than vehicle positions: a vehicle
% can be laterally offset, and its trail must not hide a nearby new leg.
% Exempt the newest 20 cm to preserve genuine corner overlap. This bounded
% history contains camera observations, not a supplied layout.
admissible=true(1,8);adjacentLaneAccepted=false(1,8);
rawCount=0;rawMinimum=1;minDir=heading;
for b=1:8
    if confidence(b)>.20
        rawCount=rawCount+1;
        continuity=dot(branchDirections(:,b),heading);
        if continuity<rawMinimum,rawMinimum=continuity;minDir=branchDirections(:,b);end
    end
end
% A very acute return can share paint with the incoming stripe until a fork
% becomes visible. Inspect that fork instead of blindly locking to old paint.
forwardOld=false(1,8);hasUnvisitedExit=false;
returnHistoryDistance=ones(1,8)*1e6;
if useHistory && distance>.8
    for b=1:8
        if confidence(b)>.20
            continuity=dot(branchDirections(:,b),heading);
            supportLow=1e6;supportHigh=-1e6;
            for k=1:384
                if k<=trailCount && distance-trailS(k)>.60
                    delta=trailXY(:,k)-branchPoints(:,b);
                    along=dot(delta,branchDirections(:,b));
                    across=abs(delta(1)*branchDirections(2,b)-delta(2)*branchDirections(1,b));
                    if across<.060 && abs(along)<.35 && ...
                            dot(branchDirections(:,b),trailDirections(:,k))<-.85
                        supportLow=min(supportLow,along);supportHigh=max(supportHigh,along);
                    end
                    separationFromHistory=norm(delta);
                    returnHistoryDistance(b)=min(returnHistoryDistance(b),separationFromHistory);
                    if continuity>.75 && separationFromHistory<.070 && ...
                            dot(branchDirections(:,b),trailDirections(:,k))<-.85
                        forwardOld(b)=true;
                    end
                end
            end
            forwardOld(b)=forwardOld(b) && supportHigh-supportLow>.35;
        end
    end
end

if useHistory && distance>.35
    for b=1:8
        continuity=dot(branchDirections(:,b),heading);
        % A straight exit at a recognized revisited junction may also
        % be old. Use the dense observed trail there; ordinary forward
        % travel retains its existing exemption from history rejection.
        if confidence(b)>.20 && (continuity<.94 || (cycleActive && rawCount>=3))
            recent=.20;historyTolerance=.070;
            % The most backward candidate can be a recent incoming
            % stripe on a short terminal leg. Check that observed
            % trail after 10 cm; distinct outgoing turns retain the
            % separate acute-corner protection below.
            if rawCount>=2 && continuity<-.50 && continuity<=rawMinimum+.01
                recent=.10;
            end
            % At an acute turn, adjacent stripes overlap near the vertex.
            % Protect only the distinct outgoing branch; the most reverse
            % incoming branch retains the normal exclusion and penalty.
            if rawCount>=2 && continuity<-.50 && continuity>rawMinimum+.01
                separation=sqrt(max(.01,1-continuity*continuity));
                recent=max(.10,min(.18,.06/separation));
                if norm(branchPoints(:,b)-observedAnchor)>.18 && confidence(b)>=.70 && ...
                        dot(branchDirections(:,b),minDir)<.97
                    historyTolerance=.030;
                end
            end
            for k=1:384
                if k<=trailCount && distance-trailS(k)>recent
                    separationFromHistory=norm(branchPoints(:,b)-trailXY(:,k));
                    historyDistance(b)=min(historyDistance(b),separationFromHistory);
                    % The narrow acute-turn tolerance protects only the recent
                    % incoming leg, never a different previously traversed section.
                    limit=historyTolerance;
                    if distance-trailS(k)>.60,limit=max(limit,.070);end
                    % Adjacent opposite-direction paint is a new hairpin lane,
                    % while a same-direction old lane or an exact reverse remains old.
                    oppositeAdjacent=rawCount>=2 && continuity<-.50 && ...
                        continuity>rawMinimum+.001 && confidence(b)>=.70 && ...
                        norm(branchPoints(:,b)-observedAnchor)>.18 && ...
                        dot(branchDirections(:,b),minDir)<.9975 && ...
                        separationFromHistory>=.030 && ...
                        dot(branchDirections(:,b),trailDirections(:,k))<-.80;
                    if separationFromHistory<limit && ~oppositeAdjacent,admissible(b)=false;end
                    if separationFromHistory<limit && oppositeAdjacent,adjacentLaneAccepted(b)=true;end
                end
            end
        end
    end
end

% At a revisited junction, prefer an observed unvisited exit. On ordinary
% two-exit lane pixels continue along the loop while inspecting for a branch;
% allow one additional circuit before a controlled hold if no exit exists.
if cycleActive && rawCount>=3
    for b=1:8
        if confidence(b)>.20 && dot(branchDirections(:,b),heading)>-.90
            for k=1:256
                if k<=cycleCount && distance-cycleS(k)>.6 && ...
                        norm(branchPoints(:,b)-cycleXY(:,k))<.13 && ...
                        dot(branchDirections(:,b),cycleDirs(:,k))>.50
                    admissible(b)=false;
                end
            end
        end
    end
end

% Never reject the only visible continuation in an overlapping corridor.
% Once a distinct unvisited exit is observed, stop preferring the old lane.
for b=1:8
    if confidence(b)>.20 && admissible(b) && dot(branchDirections(:,b),heading)>.20 && ...
            returnHistoryDistance(b)>.090
        hasUnvisitedExit=true;
    end
end
if rawCount>=2 && hasUnvisitedExit
    for b=1:8
        if forwardOld(b),admissible(b)=false;end
    end
end

% Evaluate all visible branches using Heading Continuity
bestScore=-1e9;
bestBranch=0;

% Check if any forward branch exists (continuation ahead along path of motion)
hasForward=false;
minCont=1.0;
validCount=0;
for b=1:8
    if confidence(b)>0.20 && admissible(b)
        c=dot(branchDirections(:,b),heading);
        if c>0.75, hasForward=true; end
        if c<minCont, minCont=c; end
        validCount=validCount+1;
    end
end

for b=1:8
    if confidence(b)>0.20 && admissible(b)
        d=branchDirections(:,b);
        continuity=dot(d,heading);
        side=heading(1)*d(2)-heading(2)*d(1);
        
        score=continuity+0.02*side;
        
        % When forward continuation exists, strictly penalize any non-forward branch
        if hasForward && continuity<0.15
            score=score-50.0;
        end
        
        % Use the original observed topology before history filtering. Removing
        % the incoming branch must not turn a genuine obtuse exit into a dead end.
        % When multiple branches exist at a corner, the one closest to -1.00 is the incoming line.
        if rawCount>1 && continuity<.20 && (continuity<=rawMinimum+.001 || dot(d,minDir)>.9975)
            score=score-50.0; % penalize the incoming backtrack
        elseif rawCount==1 && continuity<.20
            score=score-50.0; % single incoming branch at dead end / terminus
        end
        
        if score>bestScore
            bestScore=score;
            bestBranch=b;
        end
    end
end

% Decision logic:
% - If any valid forward, lateral, or sharp hairpin branch exists (bestScore > -1.5): take it!
% - If only a pure reverse incoming branch exists (bestScore <= -1.5): line has ended (End of Track).
if bestBranch>0 && bestScore>-1.5
    selected=bestBranch;
    lockedDirection=branchDirections(:,selected);
    % A backward border chord close to the observed old stripe is ambiguous.
    % Ask vision for local exits; do not apply this override to a distinct
    % hairpin exit far from the previously observed route.
    refineOldExit=cycleActive || (rawCount==1 && forwardOld(selected)) || ...
        (dot(lockedDirection,heading)<-.50 && historyDistance(selected)<.10 && ~adjacentLaneAccepted(selected));
    
    % Track section transitions when a distinct corner turn is negotiated
    turn=acos(max(-1,min(1,dot(heading,lockedDirection))));
    if turn>0.50 && lastTurnAngle<=0.50
        sectionCount=min(20,sectionCount+1);
    end
    lastTurnAngle=turn;
else
    % End of track: no forward continuation exists
    selected=0;
    lockedDirection=heading;
    refineOldExit=cycleActive || (rawCount>1 && min(historyDistance)<.10);
end

if cycleActive && distance-cycleStartS>cycleLength+.60
    selected=0;routeStatus=2;lockedDirection=heading;refineOldExit=false;
end

end
