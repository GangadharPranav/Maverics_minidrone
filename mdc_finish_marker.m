function [marker,valid,maxD] = mdc_finish_marker(mask,dist,scale,ringCos,ringSin)
%#codegen
% Test separate local red peaks. A disk requires a fully observed outline.
% No arena coordinates or stored track layout enter this detector.
[h,w]=size(mask);
marker=[(double(w)+1)/2;(double(h)+1)/2]; valid=false; maxD=0;
peaks=zeros(2,8); bestQuality=-1; exclusion=(.085/scale)^2;
for attempt=1:8
    peak=0; px=2; py=2;
    for x=2:w-1
        for y=2:h-1
            value=double(dist(y,x));
            if value>peak
                separate=true;
                for j=1:8
                    if j<attempt && (x-peaks(1,j))^2+(y-peaks(2,j))^2<exclusion
                        separate=false;
                    end
                end
                if separate, peak=value; px=x; py=y; end
            end
        end
    end
    peaks(:,attempt)=[px;py];
    if attempt==1, maxD=peak; marker=[double(px);double(py)]; end
    if peak*scale<.065, break; end
    if peak*scale>.14, continue; end
    % Keep refinement local to this peak; never average disconnected shapes.
    total=0; sumPoint=zeros(2,1); refine=max(2,ceil(.04/scale));
    for x=max(2,px-refine):min(w-1,px+refine)
        for y=max(2,py-refine):min(h-1,py+refine)
            value=double(dist(y,x));
            if value>=max(2,peak-.75) && (x-px)^2+(y-py)^2<=refine^2
                weight=value^2; total=total+weight;
                sumPoint=sumPoint+weight*[double(x);double(y)];
            end
        end
    end
    candidate=[double(px);double(py)];
    if total>0, candidate=sumPoint/total; end
    hits=zeros(4,1); counts=zeros(4,1); outer=false(64,1);
    radii=[.050;.075;.088;.125]/scale;
    for k=1:64
        for r=1:4
            x=round(candidate(1)+radii(r)*ringCos(k));
            y=round(candidate(2)+radii(r)*ringSin(k));
            if x>=1 && x<=w && y>=1 && y<=h
                counts(r)=counts(r)+1; hits(r)=hits(r)+double(mask(y,x));
                if r==4, outer(k)=mask(y,x); end
            end
        end
    end
    ratio=hits./max(1,counts); branches=0;
    for k=1:64
        if outer(k) && ~outer(mod(k-2,64)+1), branches=branches+1; end
    end
    credible=all(counts==64) && ...
        ratio(1)>=.90 && ratio(2)>=.85 && ratio(3)>=.85 && ...
        ratio(4)<=.12 && branches<=1;
    quality=ratio(1)+ratio(2)+ratio(3)-2*ratio(4);
    if credible && quality>bestQuality
        marker=candidate; valid=true; bestQuality=quality;
    end
end
end
