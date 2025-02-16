clc;clear ;
data_source=load('./poses/00.txt');
gt_data=csvread('./gt_loop/GT_loop_00.csv');
num=length(data_source);
GTposes=zeros(length(data_source),2); height=ones(num,1);

for index=1:length(gt_data)
height(gt_data(index,1),1)=3;
end
%%look for closure
for index=1:num
    GTposes(index,:)=[data_source(index,4),data_source(index,12)];
end
plot3(GTposes(1:end,1),GTposes(1:end,2),height,'LineWidth',2);hold on;
 zlim([0, 4]); 
 grid on;  hold on;
 
for index=1:length(gt_data)
 pose1=[GTposes(gt_data(index,1),1:2),3];
 pose2=[GTposes(gt_data(index,2),1:2),1];
 pose=[pose1;pose2];
 line(pose(:,1),pose(:,2),pose(:,3),'color','green');
end





