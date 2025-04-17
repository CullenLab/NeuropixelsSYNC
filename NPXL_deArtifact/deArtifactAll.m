%% De-artifact for ONE example channel
function [tempAll,dataArrayD]=deArtifactAll(dataArray,seg,ind,params)

dataArrayD=zeros(size(dataArray));
tempAll=zeros(size(dataArray,1),size(seg,2));

% Parameters
ll=params.ll; %how long to subtract the template for
off=params.off; %offset accounting for the "pre" period in the template
st=params.st; %difference from actual stim time and rising edge of trigger (This is typically 0.) 
% ss=params.ss; %Further offset

for k=0:383 %for each channel

p=squeeze(seg(:,:,k+1));

%Artifact template without DC offset
temp=mean(p)-mean(mean(p)); %Artifact template

s=dataArray(k+1,:); %deArtifact'ed trace

for j=1:length(ind) % for each pulse
        s(ind(j)+st:ind(j)+st+ll)=s(ind(j)+st:ind(j)+st+ll)-temp(st+off-ss:st+off+ll-ss);
end

dataArrayD(k+1,:)=s;
tempAll(k+1,:)=temp;

end

% Add last digital ch
dataArrayD(385,:)=dataArray(385,:);

end
