function [SFM,gtpose_xy] = Initialize_data(data_dir,pose_dir,dim ,range)
%%
num_rings = dim(1);
num_sectors = dim(2);
%%
lidar_data_dir = strcat(data_dir, 'velodyne/');
data_names = osdir(lidar_data_dir);
%% gps to xyz
gtpose = csvread(strcat(data_dir,pose_dir));
gtpose_xy = gtpose(:, [4,12]);
%%
num_data = length(data_names);
SFM = cell(1, num_data);
index=1;
for data_idx = 1:num_data
      
    file_name = data_names{data_idx};
    data_path = strcat(lidar_data_dir, file_name);
    
    ptcloud = Read_Bin(data_path);
    current_SFM= Generate_descriptor(ptcloud,num_sectors, num_rings,range); 
    
    % save SFM
    SFM{index} = current_SFM;   
    index=index+1;
    % display processing
    if(rem(data_idx, 100) == 0)
        message = strcat(num2str(data_idx), " / ", num2str(num_data));
        disp(message); 
    end
end
end
