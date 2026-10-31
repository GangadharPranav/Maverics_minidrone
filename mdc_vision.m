function [anchor, target, marker, visible, markerValid, direction, quality, navigation, branches, approachMarker, approachValid, lineAxes] = mdc_vision(R,G,B,height,previousDirection,refineOldExit)
%#codegen
% Camera-only red-track perception. Coordinates are [column; row] pixels.
% No track geometry, scene state, or known waypoint list is used.
% Reports ALL branches and lets the route selector/guidance choose the turn.
if nargin<6,refineOldExit=false;end
[h,w] = size(R);
h=floor(h/2); w=floor(w/2);
branches=zeros(6,8);lineAxes=zeros(5,8); % original-image target XY, direction XY, confidence, bend
persistent ringCos ringSin markerCos markerSin
if isempty(ringCos)
    ringCos=zeros(96,1); ringSin=zeros(96,1);
    markerCos=zeros(64,1); markerSin=zeros(64,1);
    for k=1:96
        a=2*pi*(k-1)/96; ringCos(k)=cos(a); ringSin(k)=sin(a);
    end
    for k=1:64
        a=2*pi*(k-1)/64; markerCos(k)=cos(a); markerSin(k)=sin(a);
    end
end
mask = false(h,w);
dist = zeros(h,w,'single');
redCount = 0;
for x=2:w-1
    for y=2:h-1
        votes=0;
        for dy=0:1
            for dx=0:1
                r=single(R(2*y-dy,2*x-dx)); g=single(G(2*y-dy,2*x-dx)); b_=single(B(2*y-dy,2*x-dx));
                votes=votes+double(r>45 && r>1.45*g && r>1.45*b_ && r-max(g,b_)>28);
            end
        end
        yes = votes>=2;
        mask(y,x)=yes;
        if yes, dist(y,x)=single(h+w); redCount=redCount+1; end
    end
end
% Two-pass chamfer distance finds the interior of the stripe and disk.
for x=2:w-1
    for y=2:h-1
        if mask(y,x)
            dist(y,x)=min(dist(y,x),min(min(dist(y-1,x)+1,dist(y,x-1)+1), ...
                min(dist(y-1,x-1)+single(1.414214),dist(y+1,x-1)+single(1.414214))));
        end
    end
end
for x=w-1:-1:2
    for y=h-1:-1:2
        if mask(y,x)
            dist(y,x)=min(dist(y,x),min(min(dist(y+1,x)+1,dist(y,x+1)+1), ...
                min(dist(y+1,x+1)+single(1.414214),dist(y-1,x+1)+single(1.414214))));
        end
    end
end
center=[(double(w)+1)/2;(double(h)+1)/2];
anchor=center; target=center; marker=center; approachMarker=2*center-.5; approachValid=false;
direction=previousDirection;
visible=redCount>=8; markerValid=false; quality=0;
navigation=zeros(3,1); % branch count and selected direction continuity
if ~visible
    anchor=2*anchor-.5; target=2*target-.5; marker=2*marker-.5; return;
end
% Detect the finish before choosing the stripe anchor. A valid detached disk
% must not become the active line just because it enters the nadir corridor.
metresPerPixel=2*max(0.12,height)/(120/(2*tan(19*pi/180)));
[marker,markerValid,maxD]=mdc_finish_marker(mask,dist,metresPerPixel,markerCos,markerSin);
if markerValid
    trackCount=0;excludeRadius2=(.115/metresPerPixel)^2;
    for x=2:w-1
        for y=2:h-1
            if (double(x)-marker(1))^2+(double(y)-marker(2))^2<=excludeRadius2
                mask(y,x)=false;dist(y,x)=0;
            end
            if mask(y,x),trackCount=trackCount+1;end
        end
    end
    redCount=trackCount;
    if trackCount<8
        visible=true;quality=1;
        anchor=2*anchor-.5;target=2*marker-.5;marker=2*marker-.5;return;
    end
end
% Ego-centric track corridor: centered on the drone's nadir (center).
% Evaluates line pixels along travel heading, rejecting lateral distractions.
dirLen=norm(previousDirection);
if dirLen>0.1, pDir=previousDirection/dirLen; else, pDir=[0;-1]; end
latDir=[-pDir(2); pDir(1)];

best=1e20; seedX=round(center(1)); seedY=round(center(2));
foundLocal=false;
for x=2:w-1
    for y=2:h-1
        d=double(dist(y,x));
        if d>0
            deltaX=double(x)-center(1);
            deltaY=double(y)-center(2);
            s=deltaX*pDir(1)+deltaY*pDir(2);
            eLat=abs(deltaX*latDir(1)+deltaY*latDir(2));
            if eLat<=8.5 && abs(s)<=28
                cost=eLat+0.30*abs(s)-0.25*min(d,3.5);
                if cost<best
                    best=cost; seedX=x; seedY=y; foundLocal=true;
                end
            end
        end
    end
end
if ~foundLocal
    best=1e20;
    for x=2:w-1
        for y=2:h-1
            d=double(dist(y,x));
            if d>0
                cost=hypot(double(x)-center(1),double(y)-center(2))-0.25*min(d,3.5);
                if cost<best
                    best=cost; seedX=x; seedY=y;
                end
            end
        end
    end
end
anchor=[double(seedX);double(seedY)];
% Smooth the ridge estimate without pulling toward distant track branches.
sumW=0; sumP=zeros(2,1);
for x=max(2,round(anchor(1))-4):min(w-1,round(anchor(1))+4)
    for y=max(2,round(anchor(2))-4):min(h-1,round(anchor(2))+4)
        weight=double(dist(y,x))^4;
        sumW=sumW+weight; sumP=sumP+weight*[x;y];
    end
end
if sumW>0, anchor=sumP/sumW; end

% Clipped-circle geometry supports horizontal acquisition only. The strict
% detector above remains the sole authority for alignment/descent.
if ~markerValid && height>.30
    [partialPoint,approachValid]=mdc_partial_marker(mask,false(h,w),metresPerPixel,markerCos,markerSin);
    if approachValid, approachMarker=2*partialPoint-.5; end
end

% Lookahead ring intersects the red stripe. Capped to image boundaries.
% A wider sensing ring helps separate adjacent legs of a hairpin. Steering
% distance is capped separately by guidance; this is only visual evidence.
radius=min(0.09/metresPerPixel, ...
    min([anchor(1)-3,w-anchor(1)-2,anchor(2)-3,h-anchor(2)-2]));
radius=max(6,min(12,radius));

% Check whether the currently observed stripe continues along the prior heading.
% If red tape continues directly ahead of anchor along pDir, the ego-lane is open!
% Follow the active line directly without looking for turns or adjacent tracks.
forwardHits=0;
for sVal=[6, 10, 14]
    sx=round(anchor(1)+sVal*pDir(1)); sy=round(anchor(2)+sVal*pDir(2));
    if sx>=2 && sx<=w-1 && sy>=2 && sy<=h-1
        if dist(sy,sx)>0, forwardHits=forwardHits+1; end
    end
end

if forwardHits==3 && ~refineOldExit
    % Forward lane open: lock to active ego-lane
    candidates=1;
    % Refit the visible stripe rather than freezing a corner-to-border
    % chord. A short final stripe must establish its actual gap direction.
    direction=fitStraightStripe(mask,dist,anchor,pDir,metresPerPixel);
    target=anchor+min(radius,14)*direction;
    branches(:,1)=[2*target-.5; direction; 1.0; 0];
    quality=min(1,redCount/63);
else
    % Inspect connected exits when the forward stripe is interrupted.
    % Flood fill connected track to trace perimeter exits and find outgoing turns.
    reachable=false(h,w);
    queueX=zeros(h*w,1,'int32'); queueY=zeros(h*w,1,'int32');
    head=1; tail=1; queueX(1)=int32(seedX); queueY(1)=int32(seedY);
    reachable(seedY,seedX)=true;
    while head<=tail
        qx=double(queueX(head)); qy=double(queueY(head)); head=head+1;
        for dx=-1:1
            for dy=-1:1
                nx=qx+dx; ny=qy+dy;
                if nx>=2 && nx<=w-1 && ny>=2 && ny<=h-1
                    if mask(ny,nx) && ~reachable(ny,nx)
                        tail=tail+1; queueX(tail)=int32(nx); queueY(tail)=int32(ny);
                        reachable(ny,nx)=true;
                    end
                end
            end
        end
    end

    % Perimeter tracing along image boundaries to find all track exits
    borderX=zeros(2*w+2*h,1,'int32'); borderY=zeros(2*w+2*h,1,'int32');
    nB=0;
    for bx=2:w-1, nB=nB+1; borderX(nB)=int32(bx); borderY(nB)=int32(2); end
    for by=3:h-2, nB=nB+1; borderX(nB)=int32(w-1); borderY(nB)=int32(by); end
    for bx=w-1:-1:2, nB=nB+1; borderX(nB)=int32(bx); borderY(nB)=int32(h-1); end
    for by=h-2:-1:3, nB=nB+1; borderX(nB)=int32(2); borderY(nB)=int32(by); end

    onB=false(nB,1);
    for bi=1:nB, onB(bi)=reachable(borderY(bi),borderX(bi)); end
    startB=0;
    for bi=1:nB
        if ~onB(bi), startB=bi; break; end
    end
    candidates=0;
    if startB>0
        curSegX=zeros(64,1,'int32');
        curSegY=zeros(64,1,'int32');
        countB=0;
        for j=1:nB+1
            bi=mod(startB+j-2,nB)+1;
            if j<=nB && onB(bi)
                if countB<64
                    countB=countB+1;
                    curSegX(countB)=borderX(bi);
                    curSegY(countB)=borderY(bi);
                end
            elseif countB>0
                stripePx=max(8,min(24,round(0.10/metresPerPixel)));
                % A diagonal stripe has a long border contact even when it
                % is only one stripe. Measure perpendicular to its direction
                % before deciding that two hairpin legs have merged.
                contact=zeros(2,1);
                for kIdx=1:countB
                    contact=contact+[double(curSegX(kIdx));double(curSegY(kIdx))];
                end
                contact=contact/countB;
                contactDirection=contact-anchor;
                contactDirection=contactDirection/max(norm(contactDirection),1e-9);
                normal=[-contactDirection(2);contactDirection(1)];
                minProjection=1e10; maxProjection=-1e10;
                for kIdx=1:countB
                    projection=double(curSegX(kIdx))*normal(1)+double(curSegY(kIdx))*normal(2);
                    minProjection=min(minProjection,projection);
                    maxProjection=max(maxProjection,projection);
                end
                mergedWidth=maxProjection-minProjection;
                if mergedWidth>=1.35*stripePx && candidates<=6
                    % Wide merged contact (acute hairpin corner): split into two distinct branches
                    i1_end=max(1,round(countB*0.40));
                    i2_start=min(countB,round(countB*0.60));
                    % Branch 1 (first half)
                    sum1X=0; sum1Y=0;
                    for kIdx=1:i1_end
                        sum1X=sum1X+double(curSegX(kIdx)); sum1Y=sum1Y+double(curSegY(kIdx));
                    end
                    exitPt1=[sum1X/i1_end; sum1Y/i1_end];
                    d1=exitPt1-anchor;
                    if norm(d1)>1e-3, d1=d1/norm(d1); else, d1=previousDirection; end
                    score1=dot(d1,previousDirection);
                    tx1=round(anchor(1)+radius*d1(1)); ty1=round(anchor(2)+radius*d1(2));
                    conn1=false;
                    for dx_=-2:2, for dy_=-2:2
                        cx=tx1+dx_; cy=ty1+dy_;
                        if cx>=1 && cx<=w && cy>=1 && cy<=h && reachable(cy,cx), conn1=true; end
                    end, end
                    if conn1
                        candidates=candidates+1;
                        branches(:,candidates)=[2*exitPt1-.5; d1; 1.0; acos(max(-1,min(1,score1)))];
                    end
                    % Branch 2 (second half)
                    sum2X=0; sum2Y=0; cnt2=countB-i2_start+1;
                    for kIdx=i2_start:countB
                        sum2X=sum2X+double(curSegX(kIdx)); sum2Y=sum2Y+double(curSegY(kIdx));
                    end
                    exitPt2=[sum2X/cnt2; sum2Y/cnt2];
                    d2=exitPt2-anchor;
                    if norm(d2)>1e-3, d2=d2/norm(d2); else, d2=previousDirection; end
                    score2=dot(d2,previousDirection);
                    tx2=round(anchor(1)+radius*d2(1)); ty2=round(anchor(2)+radius*d2(2));
                    conn2=false;
                    for dx_=-2:2, for dy_=-2:2
                        cx=tx2+dx_; cy=ty2+dy_;
                        if cx>=1 && cx<=w && cy>=1 && cy<=h && reachable(cy,cx), conn2=true; end
                    end, end
                    if conn2
                        candidates=candidates+1;
                        branches(:,candidates)=[2*exitPt2-.5; d2; 1.0; acos(max(-1,min(1,score2)))];
                    end
                elseif countB>=2 && candidates<8
                    sumX=0; sumY=0;
                    for kIdx=1:countB
                        sumX=sumX+double(curSegX(kIdx)); sumY=sumY+double(curSegY(kIdx));
                    end
                    exitPt=[sumX/countB; sumY/countB];
                    d=exitPt-anchor;
                    if norm(d)>1e-3, d=d/norm(d); else, d=previousDirection; end
                    score=dot(d,previousDirection);
                    tx=round(anchor(1)+radius*d(1)); ty=round(anchor(2)+radius*d(2));
                    conn=false;
                    for dx_=-2:2, for dy_=-2:2
                        cx=tx+dx_; cy=ty+dy_;
                        if cx>=1 && cx<=w && cy>=1 && cy<=h && reachable(cy,cx), conn=true; end
                    end, end
                    if conn
                        candidates=candidates+1;
                        branches(:,candidates)=[2*exitPt-.5; d; min(1,countB/4); acos(max(-1,min(1,score)))];
                    end
                end
                countB=0;
            end
        end
    end

    % A connected border exit may lie beyond another bend. Its chord is
    % not an immediate lane direction if it crosses background. Keep only
    % visually supported straight chords and resolve bends with local rings.
    supportedBranches=zeros(6,8);supportedCount=0;unsupportedChord=false;
    for borderCandidate=1:min(candidates,8)
        endpoint=(branches(1:2,borderCandidate)+.5)/2;
        delta=endpoint-anchor;hits=0;samples=0;
        for fraction=[.25 .50 .75 .90]
            point=round(anchor+fraction*delta);
            if point(1)>=1&&point(1)<=w&&point(2)>=1&&point(2)<=h
                samples=samples+1;hits=hits+double(mask(point(2),point(1)));
            end
        end
        supported=norm(delta)*metresPerPixel<=.14 || (samples==4&&hits>=3);
        if supported
            supportedCount=supportedCount+1;supportedBranches(:,supportedCount)=branches(:,borderCandidate);
        else
            unsupportedChord=true;
        end
    end
    branches=supportedBranches;candidates=supportedCount;

    % A short outgoing section can end INSIDE the camera frame. Border
    % exits alone then describe only the incoming/older sections. Always
    % inspect local ring exits at increasing radii as needed, retaining border lookahead for
    % genuine acute corners that merge on the smaller local ring.
    needLocal=candidates==0 || unsupportedChord;
        % Multiple image-border exits can all belong to older connected legs.
    % Their count cannot establish that the immediate outgoing leg is visible.
    bestContinuation=-1;
    for localCandidate=1:min(candidates,8)
        bestContinuation=max(bestContinuation,dot(branches(3:4,localCandidate),pDir));
    end
    if candidates==1 && bestContinuation<0,needLocal=true;end
    if refineOldExit,needLocal=true;end
    if needLocal
      initialLocalCount=candidates;
      for ringPass=1:4
        localRadius=radius;
        if ringPass>1
            extraRadii=[.13 .17 .21]/metresPerPixel;
            localRadius=min(extraRadii(ringPass-1),min([anchor(1)-3,w-anchor(1)-2,anchor(2)-3,h-anchor(2)-2]));
            if localRadius<radius+2,continue;end
        end
        onRing=false(96,1);
        for k=1:96
            x=round(anchor(1)+localRadius*ringCos(k)); y=round(anchor(2)+localRadius*ringSin(k));
            if x>=1 && x<=w && y>=1 && y<=h, onRing(k)=reachable(y,x); end
        end
        start=0;
        for k=1:96
            if ~onRing(k), start=k; break; end
        end
        if start>0
            sumArc=zeros(2,1); arcCount=0;
            for j=1:97
                k=mod(start+j-1,96)+1;
                if j<=96 && onRing(k)
                    sumArc=sumArc+[ringCos(k);ringSin(k)]; arcCount=arcCount+1;
                elseif arcCount>0
                    d=sumArc/max(norm(sumArc),1e-9);
                    contHits=0;
                    probeSign=1;if ringPass>1,probeSign=-1;end
                    for stepVal=probeSign*[3, 6]
                        cx=round(anchor(1)+(localRadius+stepVal)*d(1));
                        cy=round(anchor(2)+(localRadius+stepVal)*d(2));
                        if cx>=2 && cx<=w-1 && cy>=2 && cy<=h-1
                            if dist(cy,cx)>0, contHits=contHits+1; end
                        end
                    end
                    duplicate=false;
                    % A forward local exit can be missing from the border
                    % candidates when only the incoming line reaches the
                    % image edge. Deduplicate against actual directions
                    % below; another branch alone is not a duplicate.
                    for bc=1:min(candidates,8)
                        if dot(d,branches(3:4,bc))>.97, duplicate=true; end
                    end
                    if arcCount>=2 && contHits>=1 && candidates<8 && ~duplicate
                        candidates=candidates+1;
                        score=dot(d,previousDirection);
                        branches(:,candidates)=[2*(anchor+localRadius*d)-.5;d;min(1,arcCount/4);acos(max(-1,min(1,score)))];
                    end
                    sumArc(:)=0; arcCount=0;
                end
            end
        end
        if candidates>=2 && candidates>initialLocalCount,break;end
      end
    end

    if candidates==0
        if markerValid || maxD*metresPerPixel>=0.065
            target=marker;
        else
            target=anchor;
        end
        quality=0.2;
        direction=previousDirection;
    elseif candidates==1
        score=dot(branches(3:4,1),previousDirection);
        if score<=-0.982
            if markerValid || maxD*metresPerPixel>=0.060
                target=marker;
            else
                target=anchor;
            end
            direction=previousDirection;
            quality=0.5;
        else
            direction=branches(3:4,1);
            target=anchor+radius*direction;
            quality=min(1,redCount/63);
        end
    else
        % Multiple candidate branches: pick the best outgoing turn (penalize exact reversal <= -0.982)
        bestScore=-1e10; bestCross=-1e10;
        for bc=1:min(candidates,8)
            d=branches(3:4,bc);
            score=dot(d,previousDirection);
            side=previousDirection(1)*d(2)-previousDirection(2)*d(1);
            effectiveScore=score;
            if score<=-0.982
                effectiveScore=score-50.0; % strictly penalize exact reverse
            end
            if effectiveScore>bestScore+0.025 || (abs(effectiveScore-bestScore)<=0.025 && side>bestCross)
                bestScore=effectiveScore; bestCross=side; direction=d; target=anchor+radius*d;
            end
        end
        quality=min(1,redCount/63);
    end
end
% Fit isolated visible branch cross-sections, away from the broad junction.
for axisIndex=1:8
 if branches(5,axisIndex)>.20
  [axisPoint,axisDirection,axisQuality]=fitBranchAxis(mask,dist,anchor,branches(3:4,axisIndex),metresPerPixel);
  lineAxes(:,axisIndex)=[2*axisPoint-.5;axisDirection;axisQuality];
 end
end
aheadClearance=visibleForwardClearance(mask,anchor,pDir,metresPerPixel);
navigation=[double(candidates);dot(direction,previousDirection);aheadClearance];
anchor=2*anchor-.5; target=2*target-.5; marker=2*marker-.5;
end

function direction=fitStraightStripe(mask,dist,anchor,prior,scale)
% Five connected cross-sections; reject broad junctions and curved profiles.
% Fixed arrays/loops. This estimates the local stripe, not a future waypoint.
[h,w]=size(mask);normal=[-prior(2);prior(1)];direction=prior;
ss=zeros(5,1);ll=zeros(5,1);n=0;maximumInterior=0;
for sample=1:5
 s=4*(sample-1);point=anchor+s*prior;
 cx=round(point(1));cy=round(point(2));
 if cx<2||cx>w-1||cy<2||cy>h-1||~mask(cy,cx),continue;end
 lower=0;upper=0;
 for step=1:12
  x=round(point(1)-step*normal(1));y=round(point(2)-step*normal(2));
  if x<2||x>w-1||y<2||y>h-1||~mask(y,x),break;end
  lower=step;
 end
 for step=1:12
  x=round(point(1)+step*normal(1));y=round(point(2)+step*normal(2));
  if x<2||x>w-1||y<2||y>h-1||~mask(y,x),break;end
  upper=step;
 end
 if (lower+upper)*scale>.115,continue;end
 total=0;offset=0;
 for step=-lower:upper
  x=round(point(1)+step*normal(1));y=round(point(2)+step*normal(2));
  maximumInterior=max(maximumInterior,double(dist(y,x))*scale);
  weight=double(dist(y,x))^4;total=total+weight;offset=offset+weight*step;
 end
 if total>0,n=n+1;ss(n)=s;ll(n)=offset/total;end
end
% A merged hairpin is wider than a single stripe; do not fit its bisector.
if n<4||ss(n)-ss(1)<12||maximumInterior>.058,return;end
meanS=sum(ss(1:n))/n;meanL=sum(ll(1:n))/n;variance=0;covariance=0;
for k=1:5
 if k<=n,variance=variance+(ss(k)-meanS)^2;covariance=covariance+(ss(k)-meanS)*(ll(k)-meanL);end
end
slope=covariance/max(1e-9,variance);residual=0;
for k=1:5
 if k<=n,residual=residual+(ll(k)-meanL-slope*(ss(k)-meanS))^2;end
end
if sqrt(residual/n)*scale<=.008 && abs(slope)<=.85
 direction=prior+slope*normal;direction=direction/max(1e-9,norm(direction));
end
end

function clearance=visibleForwardClearance(mask,anchor,prior,scale)
% Preview only the currently visible stripe. Image clipping is unknown, not a turn.
[h,w]=size(mask);clearance=.50;lastRed=0;missing=0;
for step=0:2:80
 point=round(anchor+step*prior);
 if point(1)<2||point(1)>w-1||point(2)<2||point(2)>h-1,return;end
 if mask(point(2),point(1)),lastRed=step;missing=0;else,missing=missing+1;end
 if missing>=2,clearance=max(0,lastRed*scale);return;end
end
end

function [point,direction,quality]=fitBranchAxis(mask,dist,anchor,prior,scale)
% Fixed-size observed centreline fit; no map, waypoint or goal is consulted.
[h,w]=size(mask);normal=[-prior(2);prior(1)];point=anchor;direction=prior;quality=0;
ss=zeros(7,1);ll=zeros(7,1);n=0;
for sample=1:7
 s=(.04+.025*(sample-1))/scale;p=anchor+s*prior;
 x=round(p(1));y=round(p(2));
 if x<2||x>w-1||y<2||y>h-1||~mask(y,x),continue;end
 lower=0;upper=0;closedLower=false;closedUpper=false;
 for step=1:12
  px=round(p(1)-step*normal(1));py=round(p(2)-step*normal(2));
  if px<2||px>w-1||py<2||py>h-1,break;end
  if ~mask(py,px),closedLower=true;break;end
  lower=step;
 end
 for step=1:12
  px=round(p(1)+step*normal(1));py=round(p(2)+step*normal(2));
  if px<2||px>w-1||py<2||py>h-1,break;end
  if ~mask(py,px),closedUpper=true;break;end
  upper=step;
 end
 if ~closedLower||~closedUpper||(lower+upper)*scale>.115,continue;end
 total=0;offset=0;
 for step=-lower:upper
  px=round(p(1)+step*normal(1));py=round(p(2)+step*normal(2));
  weight=double(dist(py,px))^4;total=total+weight;offset=offset+weight*step;
 end
 if total>0,n=n+1;ss(n)=s;ll(n)=offset/total;end
end
if n<3||(ss(n)-ss(1))*scale<.045,return;end
meanS=sum(ss(1:n))/n;meanL=sum(ll(1:n))/n;variance=0;covariance=0;
for k=1:7
 if k<=n,variance=variance+(ss(k)-meanS)^2;covariance=covariance+(ss(k)-meanS)*(ll(k)-meanL);end
end
slope=covariance/max(variance,1e-9);residual=0;
for k=1:7
 if k<=n,residual=residual+(ll(k)-meanL-slope*(ss(k)-meanS))^2;end
end
if sqrt(residual/n)*scale>.005||abs(slope)>.50,return;end
point=anchor+meanS*prior+meanL*normal;
direction=prior+slope*normal;direction=direction/max(1e-9,norm(direction));quality=n/7;
end
