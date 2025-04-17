clear

%% Parameters

params.pre=1; % How many milliseconds to visualize (and make template for) before the start of each pulse
params.post=5; % How many milliseconds to visualize (and make template for) after the start of each pulse
params.trigTresh=10; %Threshold for triggers 

params.ll=30; %how long to subtract the template for
params.off=params.pre*30+1; %offset accounting for the "pre" period in the template
params.st=0; %difference from actual stim time and rising edge of trigger (This is typically 0.) 
params.ss=0; %Further offset

params.k=339; % Channel to visualize

% Specity file name and location 
%  Adapted partly from SpikeGLX_Datafile_Tools by Jennifer Colonell

% Ask user for binary file
[binName,path] = uigetfile('*.bin', 'Select Binary File');

% Parse the corresponding metafile
meta = SGLX_readMeta.ReadMeta(binName, path);

% Specify time point to load

bt=0; %Sample number to start from
et=15; %How many seconds of data to load (30 kHz sampling rate)

% Get data
dataArray = SGLX_readMeta.ReadBin(30000*bt,30000*(et-bt), meta, binName, path);

% Plot trigger channel
figure(12)
plot(dataArray(385,:))

% Grab segments

ind=findTrigRisingEdge(dataArray(385,:),params.trigTresh);

seg=grabSegments(dataArray,ind,params.pre,params.post);

% Plot the data aligned to each stimulation pulse
figure(5)
plot(squeeze(seg(:,:,1)'))

% One channel
[temp,out,outf]=deArtifact1Ch(dataArray,seg,ind,params);

%% De-artifact for each channel
[tempAll,dataArrayD]=deArtifactAll(dataArray,seg,ind,params);

%% Write bin file output (with D added to the end of file name)
WriteBin(dataArrayD,[binName(1:end-13) 'D.imec0.ap.bin'],'.')

%% Save mat file 
data=out;
trig=dataArray(385,:);
save([binName(1:end-15) '.mat'], 'data','trig','params','tempAll')
