function  [angle,min_dis]=align_rim(rim1,rim2)
  dis_temp=0;
for j=0:119
     rim= circshift(rim2,j,2);
     dis = dot(rim, rim1) / (norm(rim)*norm(rim1));
%      dis=sum(abs(rim1-rim))/120;
     if dis>dis_temp
         dis_temp=dis;
         angle=j;
     end
end
 min_dis=dis_temp;
end