 function rim=view_rim(des)
 col=size(des,2);
 rim=zeros(1,col);
    for j=1:col
       a=find(des(:,j),1, 'last'); 
       
       if ~isempty(a)
         rim(1,j)=a;
       end
    end
end
