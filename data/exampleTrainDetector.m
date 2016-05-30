%The following code was used to train the attached detectors.

PMTroot = 'G:\Eshed\pmt\';
addpath(genpath(PMTroot))

dataDir = 'G:\Eshed\HandDataReduced\';
%%
% Train a generic hand detector
%
% set up opts for training detector (see acfTrain)

opts=acfTrain(); opts.modelDs=[50 41]; opts.modelDsPad=[64 64];
opts.posGtDir=[dataDir 'train/posGt']; opts.nWeak=[32 128 512 2048];
opts.posImgDir=[dataDir 'train/pos']; opts.pJitter=struct('flip',0);
opts.pBoost.pTree.fracFtrs=1/16;
opts.pNms.overlap = 0.2;
opts.pPyramid.pChns.shrink = 2;
opts.name='sampleDetector';
detector = acfTrain( opts );

%%
% Evaluate a detector
%
imgNms=bbGt('getFiles',{[dataDir 'train/pos']});
 for i = 1000
I=imread(imgNms{i}); tic, bbs=acfDetect(I,detector); toc
figure(1); im(I); bbApply('draw',bbs); pause(1);
 end

%% 
% test detector and plot roc (see acfTest)
% you should setup this up for cross validation
%

opts.pLoad={'squarify',{}}; 
tic
[miss,roc,gt,dt]=acfTest('name',opts.name,'imgDir',[dataDir 'train/pos'],...
  'gtDir',[dataDir 'train/posGt'],'pLoad',opts.pLoad,'reapply',1,'show',2);
toc

%%
% Train a left/right passenger/driver hand detector
%
alllabs = {'leftHand_driver','rightHand_driver',...
    'leftHand_passenger','rightHand_passenger'};
for i_type = 1:4
    opts=acfTrain(); opts.modelDs=[50 41]; opts.modelDsPad=[64 64];
    opts.posGtDir=[dataDir 'train/posGt']; opts.nWeak=[32 128 512 2048];
    opts.posImgDir=[dataDir 'train/pos']; opts.pJitter=struct('flip',0);
    opts.pBoost.pTree.fracFtrs=1/16;
    opts.pNms.overlap = 0.2;
    opts.pPyramid.pChns.shrink = 2;
    
    switch i_type
        case 1; opts.pLoad={'lbls', {alllabs{1}}, 'ilbls',{alllabs{2},alllabs{3},alllabs{4}}};
        case 2; opts.pLoad={'lbls', {alllabs{2}}, 'ilbls',{alllabs{1},alllabs{3},alllabs{4}}};
        case 3; opts.pLoad={'lbls', {alllabs{3}}, 'ilbls',{alllabs{1},alllabs{2},alllabs{4}}};
        case 4; opts.pLoad={'lbls', {alllabs{4}}, 'ilbls',{alllabs{1},alllabs{2},alllabs{3}}};
    end
    
    opts.name=['sampleType' num2str(i_type)];
    
    detector = acfTrain( opts );
end

