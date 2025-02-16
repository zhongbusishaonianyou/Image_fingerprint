clc;clear ;
data_source=load('./poses/05.txt');
GTposes=zeros(length(data_source),2);
%%look for closure
for index=1:length(data_source)
    GTposes(index,:)=[data_source(index,4),data_source(index,12)];
end
plot(GTposes(1:end,1),GTposes(1:end,2));
count=1;
for i=301:length(GTposes)
    for j=1:i-300
        distance(j,1)=sqrt((GTposes(j,1)-GTposes(i,1))^2+(GTposes(j,2)-GTposes(i,2))^2);
    end
    [min_dist,loop_index]=min(distance);
    if(min_dist<4)
        gt_cloure_data(count,:)=[i,loop_index,min_dist];
        count=count+1;
    end
end
%% Save the result into csv file 
save_dir = strcat('./gt_loop/','/');
save_file_name = strcat(save_dir, '/GT_loop_05.csv');
if( ~exist(save_dir))
    mkdir(save_dir)
end
csvwrite(save_file_name, gt_cloure_data);
