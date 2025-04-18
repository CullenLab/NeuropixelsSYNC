%
% Copyright (c) 2025, CullenLab, Johns Hopkins University
% All rights reserved.
% Created by Pum Wiboonsaksakul
%
% This source code is licensed under the MIT-style license found in the
% LICENSE file in the root directory of this source tree. 

function ind=findTrigRisingEdge(data,thresh)

ind=find(diff(data)>thresh);

end
