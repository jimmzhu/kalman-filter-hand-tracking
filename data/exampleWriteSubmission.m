
%ADD PMT, download from http://vision.ucsd.edu/~pdollar/toolbox/doc/
PMTroot = 'G:\Eshed\pmt\';
addpath(genpath(PMTroot))
%%
%Write detections in submission format
testdir = 'G:\Eshed\HandDataReduced\test\pos\';
imnames = dir([testdir '*.png']);
Nimages = length(imnames);

%Load sample detector
detector = load('sampleGenericDetector.mat');
detector = detector.detector;
%%
bbs = cell(1,Nimages);
%Detect
parfor i = 1:length(imnames)
    I = imread([testdir imnames(i).name]);
    bbs{i} = acfDetect(I,dets);
    %imshow(I);bbApply('draw',bbs{i})
end
%%
%Write
%[imagename x y w h score left/right(0/1) driver/passenger(0/1) number_hands_on_wheel(0/1/2)];

outfile = 'myDets.txt';
fid = fopen(outfile,'w');
for i=1:length(imnames)
    currname = imnames(i).name;
    currbbs = bbs{i};
    for j = 1:size(currbbs,1)
        fprintf(fid, '%s %d %d %d %d %d %d %d %d\n',...
            currname,currbbs(j,1),currbbs(j,2),currbbs(j,3),currbbs(j,4),currbbs(j,5),-1,-1,-1);
    end
end
fclose(fid);