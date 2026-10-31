function [budget,termination,originSupported]=mdc_visible_progress(R,G,B,height,anchor,direction)
%#codegen
% Camera-only straight painted support from an observed stripe anchor.
% budget is VERIFIED metres from anchor; termination: 1 background/invalid,
% 2 image edge, 3 scan limit. Image clipping never means unknown clear space.
% Fixed-size loops and a three-pixel cross-section tolerate pixel rounding;
% no global red-area percentage, supplied geometry, map or goal is consulted.
budget=0;termination=1;originSupported=false;
[h,w]=size(R);scale=max(.12,double(height))/(120/(2*tan(19*pi/180)));
d=double(direction(:));lengthD=norm(d);
if lengthD<.1,return;end
d=d/lengthD;normal=[-d(2);d(1)];missing=0;
for step=0:2:96
    distance=double(step)*scale;
    if distance>.28,termination=3;return;end
    point=double(anchor(:))+double(step)*d;
    x=round(point(1));y=round(point(2));
    if x<2 || x>w-1 || y<2 || y>h-1,termination=2;return;end
    painted=false;
    for offset=-1:1
        pixel=round(point+double(offset)*normal);
        if pixel(1)>=1 && pixel(1)<=w && pixel(2)>=1 && pixel(2)<=h
            red=double(R(pixel(2),pixel(1)));green=double(G(pixel(2),pixel(1)));blue=double(B(pixel(2),pixel(1)));
            if red>45 && red>1.45*green && red>1.45*blue && red-max(green,blue)>28
                painted=true;
            end
        end
    end
    if step==0
        originSupported=painted;
        if ~originSupported,return;end
    end
    if painted
        budget=distance;missing=0;
    else
        missing=missing+1;
        if missing>=2,termination=1;return;end
    end
end
termination=3;
end
