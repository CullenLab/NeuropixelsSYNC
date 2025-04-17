function seg=grabSegments1Ch(data,ind,pre,post)

tnum=length(ind);
nnum=(pre+post)*30+1;
seg=zeros(tnum,nnum);

    for j=1:tnum
    seg(j,:)=data(ind(j)-pre*30:ind(j)+post*30);
    end


end