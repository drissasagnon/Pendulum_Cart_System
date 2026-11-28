v = VideoReader('media/demo.mp4');
gifFile = 'media/demo.gif';
step = max(1, round(v.FrameRate/10));      % keep ~10 frames per second
k = 0; first = true;
while hasFrame(v)
    frame = readFrame(v);  k = k + 1;
    if mod(k-1, step) ~= 0, continue; end
    [A, map] = rgb2ind(frame, 64);         % 64 colors keeps the file small
    if first
        imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 0.1);
        first = false;
    else
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
