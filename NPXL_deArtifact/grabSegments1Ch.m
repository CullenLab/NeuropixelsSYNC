%
% Copyright (c) 2025, CullenLab, Johns Hopkins University
% All rights reserved.
% Created by Pum Wiboonsaksakul
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree. 

function seg=grabSegments1Ch(data,ind,pre,post)

tnum=length(ind);
nnum=(pre+post)*30+1;
seg=zeros(tnum,nnum);

    for j=1:tnum
    seg(j,:)=data(ind(j)-pre*30:ind(j)+post*30);
    end


end
