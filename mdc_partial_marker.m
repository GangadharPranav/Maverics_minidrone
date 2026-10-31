function [center,valid,bestQuality] = mdc_partial_marker(mask,lane,scale,cs,sn)
%#codegen
% A clipped circular boundary may guide APPROACH ONLY, never descent.
% Inputs use the internal vision grid; scale is metres per internal pixel.
% Unlike the old partial detector, this fits the observed natural arc rather
% than requiring a large distance-transform peak or guessing a box centre.
[h,w]=size(mask);
center=[(double(w)+1)/2;(double(h)+1)/2];
valid=false;bestQuality=-1;
radius=.10/scale;
if radius<5 || radius>min(h,w)/2.5,return;end
pending=mask;
qx=zeros(h*w,1,'int32');qy=zeros(h*w,1,'int32');
bx=zeros(h*w,1);by=zeros(h*w,1);
components=0;
for sx=2:w-1
 for sy=2:h-1
  if ~pending(sy,sx),continue;end
  components=components+1;
  if components>12,return;end
  head=1;tail=1;qx(1)=int32(sx);qy(1)=int32(sy);
  pending(sy,sx)=false;
  minX=sx;maxX=sx;minY=sy;maxY=sy;lanePixels=0;
  while head<=tail
   x=double(qx(head));y=double(qy(head));head=head+1;
   minX=min(minX,x);maxX=max(maxX,x);
   minY=min(minY,y);maxY=max(maxY,y);
   lanePixels=lanePixels+double(lane(y,x));
   for dx=-1:1
    for dy=-1:1
     nx=x+dx;ny=y+dy;
     if nx>=2 && nx<=w-1 && ny>=2 && ny<=h-1 && pending(ny,nx)
      tail=tail+1;qx(tail)=int32(nx);qy(tail)=int32(ny);
      pending(ny,nx)=false;
     end
    end
   end
  end
  % Reject ordinary/full components here; the strict full-circle detector
  % handles those. A partial cue must actually touch an image clipping edge.
  clippedEdges=double(minX<=2)+double(maxX>=w-1)+double(minY<=2)+double(maxY>=h-1);
  clipped=clippedEdges>0;
  width=maxX-minX+1;height=maxY-minY+1;
  if ~clipped || tail<.12*pi*radius^2 || tail>1.35*pi*radius^2 || ...
      max(width,height)>2.35*radius || min(width,height)<.25*radius
   continue;
  end
  % Natural red/background boundary only: do not fit the artificial straight
  % cut introduced by the camera edge. Store a bounded number of pixels.
  n=0;meanPoint=zeros(2,1);
  for j=1:tail
   x=double(qx(j));y=double(qy(j));
   if x<=3 || x>=w-2 || y<=3 || y>=h-2,continue;end
   boundary=false;
   for dx=-1:1
    for dy=-1:1
     if ~mask(y+dy,x+dx),boundary=true;end
    end
   end
   if boundary
    n=n+1;bx(n)=x;by(n)=y;meanPoint=meanPoint+[x;y];
   end
  end
  if n<max(8,ceil(.7*radius)),continue;end
  meanPoint=meanPoint/n;
  xx=0;xy=0;yy=0;xr=0;yr=0;
  for j=1:n
   dx=bx(j)-meanPoint(1);dy=by(j)-meanPoint(2);rr=dx*dx+dy*dy;
   xx=xx+dx*dx;xy=xy+dx*dy;yy=yy+dy*dy;
   xr=xr+.5*dx*rr;yr=yr+.5*dy*rr;
  end
  determinant=xx*yy-xy*xy;
  if determinant<1e-5*max(1,(xx+yy)^2),continue;end
  candidate=meanPoint+[(yy*xr-xy*yr);(xx*yr-xy*xr)]/determinant;
  fittedRadius=0;
  for j=1:n,fittedRadius=fittedRadius+hypot(bx(j)-candidate(1),by(j)-candidate(2));end
  fittedRadius=fittedRadius/n;
  % On a two-edge clip, the remaining quarter arc spans few raster pixels.
  % Its unconstrained algebraic radius is biased downwards; allow the
  % following known-radius refinement to evaluate that hypothesis instead.
  % Single-edge candidates keep the stricter lower bound. All final arc,
  % residual and whole-component overlap tests remain mandatory.
  minimumFit=.70;if clippedEdges>=2,minimumFit=.30;end
  if fittedRadius<minimumFit*radius || fittedRadius>1.25*radius,continue;end
  % Refine against the known marker radius. Boundary red pixel centres lie
  % about half a pixel inside the continuous outline. Three fixed iterations
  % prevent the poorly conditioned short arc from choosing a tiny circle.
  boundaryRadius=radius-.5;
  for iteration=1:3
   hxx=0;hxy=0;hyy=0;gx=0;gy=0;
   for j=1:n
    dx=candidate(1)-bx(j);dy=candidate(2)-by(j);distance=hypot(dx,dy);
    if distance>.1
     ux=dx/distance;uy=dy/distance;residual=distance-boundaryRadius;
     hxx=hxx+ux*ux;hxy=hxy+ux*uy;hyy=hyy+uy*uy;
     gx=gx+ux*residual;gy=gy+uy*residual;
    end
   end
   determinant=hxx*hyy-hxy*hxy;
   if determinant>1e-6
    step=[hyy*gx-hxy*gy;hxx*gy-hxy*gx]/determinant;
    stepNorm=norm(step);if stepNorm>2,step=2*step/stepNorm;end
    candidate=candidate-step;
   end
  end
  if candidate(1)<2-.80*radius || candidate(1)>w-1+.80*radius || ...
      candidate(2)<2-.80*radius || candidate(2)>h-1+.80*radius
   continue;
  end
  residualSum=0;residualMax=0;
  for j=1:n
   residual=abs(hypot(bx(j)-candidate(1),by(j)-candidate(2))-boundaryRadius);
   residualSum=residualSum+residual*residual;residualMax=max(residualMax,residual);
  end
  rms=sqrt(residualSum/n)*scale;
  if rms>.011 || residualMax*scale>.025,continue;end
  % Both sides of the observed natural arc must agree with a circular edge.
  % A straight stripe/cap must not become a pad merely because it is clipped.
  arcCount=0;arcGood=0;
  for k=1:64
   ix=round(candidate(1)+(radius-1.8)*cs(k));
   iy=round(candidate(2)+(radius-1.8)*sn(k));
   ox=round(candidate(1)+(radius+1.8)*cs(k));
   oy=round(candidate(2)+(radius+1.8)*sn(k));
   if ix>=3 && ix<=w-2 && iy>=3 && iy<=h-2 && ...
       ox>=3 && ox<=w-2 && oy>=3 && oy<=h-2
    arcCount=arcCount+1;
    if mask(iy,ix) && ~mask(oy,ox),arcGood=arcGood+1;end
   end
  end
  minimumArc=12;if clippedEdges>=2,minimumArc=11;end
  if arcCount<minimumArc || arcGood<.90*arcCount,continue;end
  % Compare the whole observed component with the visible disk, so an arc
  % attached to red tape cannot be accepted from a few convenient samples.
  predicted=0;intersection=0;outside=0;
  for x=max(2,floor(candidate(1)-radius-1)):min(w-1,ceil(candidate(1)+radius+1))
   for y=max(2,floor(candidate(2)-radius-1)):min(h-1,ceil(candidate(2)+radius+1))
    if hypot(x-candidate(1),y-candidate(2))<=radius
     predicted=predicted+1;
     if mask(y,x),intersection=intersection+1;end
    end
   end
  end
  for j=1:tail
   if hypot(double(qx(j))-candidate(1),double(qy(j))-candidate(2))>radius+1.5
    outside=outside+1;
   end
  end
  iou=intersection/max(1,predicted+tail-intersection);
  if predicted<.12*pi*radius^2 || iou<.82 || outside>.04*tail,continue;end
  % A component also selected as the current line may still be a lone disk
  % after crossing the gap. The geometric tests above, not seed identity,
  % decide; retain a small quality preference for a separate component.
  quality=iou+.1*arcGood/max(1,arcCount)-2*rms-.02*double(lanePixels>0);
  if quality>bestQuality
   center=candidate;valid=true;bestQuality=quality;
  end
 end
end
end
