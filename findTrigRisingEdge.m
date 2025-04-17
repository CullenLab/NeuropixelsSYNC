function ind=findTrigRisingEdge(data,thresh)

ind=find(diff(data)>thresh);

end