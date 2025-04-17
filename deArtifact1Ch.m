%% De-artifact for ONE example channel
function [temp,out,outf]=deArtifact1Ch(dataArray,seg,ind,params)

% Parameters
ll=params.ll; %how long to subtract the template for
off=params.off; %offset accounting for the "pre" period in the template
st=params.st; %difference from actual stim time and rising edge of trigger (This is typically 0.) 
ss=params.ss; %Further offset
k=params.k; % Channel
pre=params.pre; % How many milliseconds to visualize (and make template for) before the start of each pulse
post=params.post; % How many milliseconds to visualize (and make template for) after the start of each pulse

p=squeeze(seg(:,:,k+1));

%Artifact template without DC offset
temp=mean(p)-mean(mean(p)); %Artifact template

figure(78)
plot(temp')
title('Artifact template')

s=dataArray(k+1,:); %deArtifact'ed trace
sp=s; %Raw trace

% For each pulse, remove subtract the template out
for j=1:length(ind)
        s(ind(j)+st:ind(j)+st+ll)=s(ind(j)+st:ind(j)+st+ll)-temp(st+off-ss:st+off+ll-ss);
end

% High-pass filter at 300 Hz
order=4;
[b,a] = butter(order,300/(30000/2),'high');
sf=filtfilt(b,a,s);


% Plot the voltage traces before and after artifact removal
figure(45)
plot(sp)
hold on
plot(s)
plot(dataArray(end,:))
hold off
legend('Raw','After artifact removal','Triggers')

% Grab segments after filter
    out=grabSegments1Ch(sf,ind,params.pre,params.post);
    outf=grabSegments1Ch(s,ind,params.pre,params.post);

% Plot output
tt=((-pre*30-st):(post*30-st))/30; %Time vector for plots (ms)

figure(1000+k)

subplot(3,1,1)
% Removal period
y3=[1 1 -1 -1 1]*100;
x3=[0 ll/30 ll/30 0 0];
fill(x3,y3,[0.92 0.92 0.92],   'EdgeAlpha',0)
hold on
plot(tt,seg(:,:,k+1)')
hold off
title(['Ch = ' num2str(k) ' Original'])
xlabel('Time from the start of stimulation pulse (ms)')

subplot(3,1,2)
fill(x3,y3,[0.92 0.92 0.92],   'EdgeAlpha',0)
hold on
plot(tt,outf')
hold off
title('Artifact removed - not filtered')
xlabel('Time from the start of stimulation pulse (ms)')


subplot(3,1,3)
fill(x3,y3,[0.92 0.92 0.92],   'EdgeAlpha',0)
hold on
plot(tt,out')
hold off
title('Artifact removed - Filtered')
xlabel('Time from the start of stimulation pulse (ms)')

end
