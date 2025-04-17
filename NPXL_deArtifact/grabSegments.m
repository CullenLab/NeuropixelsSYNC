function seg=grabSegments(data,ind,pre,post)

tnum=length(ind);
nnum=(pre+post)*30+1;
seg=zeros(tnum,nnum,385);

for k=1:385
    for j=1:tnum
    seg(j,:,k)=data(k,ind(j)-pre*30:ind(j)+post*30);
    end
end

end