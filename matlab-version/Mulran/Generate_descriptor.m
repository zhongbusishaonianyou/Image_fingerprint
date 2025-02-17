function  SFM = Generate_descriptor( ptcloud, num_sectors, num_rings,max_range)
% Downsampling for better pre
downsample = 1.2; % used in the paper. 
ptcloud = pcdownsample(ptcloud, 'gridAverage', downsample);
% point cloud information 
num_points = ptcloud.Count;
SFM=zeros(num_rings,num_sectors);
%% Save a point to the corresponding bin 
for index =1:num_points
    curr_point = ptcloud.Location(index,:);
    dis= sqrt(curr_point(1)^2 + curr_point(2)^2);
    
    if (dis>max_range)
        continue;
    end      
    %structure matrix generation
     arc = (atan2(curr_point(1,3), dis) * 180.0 /pi) + 22.5;
     yaw = (atan2(curr_point(1,2), curr_point(1,1)) * 180.0/pi) + 180;
     Encoding_dis = min(max(floor(dis), 0), 79);
     %para_coding changes according to different lidar types and environmental features
     para_coding=5.6;
     Encoding_arc = min(max(floor(arc /para_coding), 0), 7);
     Encoding_yaw = min(max(floor(yaw /3), 0), 119);
     SFM(Encoding_dis+1, Encoding_yaw+1)=bitor(SFM(Encoding_dis+1, Encoding_yaw+1),bitshift(1,Encoding_arc));
end
end
