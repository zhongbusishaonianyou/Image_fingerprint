function [SFM, GTposes] = Mulran_Load_Data(dim,max_range)
%  directory structure 
% - riverside02
%   - global_pose.csv (gtpose)
%   - sensor_data
%      - outster
%        - <1564718056109702329.bin>
%        - <1564718056809740958.bin>
%      - other files
SequenceDate = 'sensor_data'; 
ScanBaseDir = 'E:\dataset/mulran/DCC03/'; 

ScanDir = strcat(ScanBaseDir, SequenceDate, '/Ouster/','Ouster/');
Scans = dir(ScanDir); Scans(1:2, :) = []; Scans = {Scans(:).name};
ScanTimes = get_scan_Information(Scans);
%%  load GT poses 
GroundTruthPosePath = strcat(ScanBaseDir, '/global_pose', '.csv');
GroundTruthPoseData = csvread(GroundTruthPosePath);
%% Start position turns to zero
GroundTruthPoseTime = GroundTruthPoseData(:, 1);
GroundTruthPoseXYZ = [GroundTruthPoseData(:, 5)-GroundTruthPoseData(1,5),GroundTruthPoseData(:,9)-GroundTruthPoseData(1,9),GroundTruthPoseData(:,13)-GroundTruthPoseData(1,13)];
%% Look for the latest timestamp with the current frame
nScanTimes = length(ScanTimes);
poses=zeros(nScanTimes,3);
for index =1:nScanTimes 
     curScanTime = ScanTimes(index, 1);
    [~, ArgminIdx] = min(abs(GroundTruthPoseTime-curScanTime));
      curgtpose = GroundTruthPoseXYZ(ArgminIdx,:);
      poses(index,:)=curgtpose;
end
%% remove zero element
count=0;
for i =1:nScanTimes 
     if(poses(i,1)==0)
         count=count+1;
     else
         break;
     end
end
GTposes=poses(count+1:end,:);
num_series=nScanTimes-count;
%%
data_idx=1;
num_rings = dim(1);num_sectors = dim(2);
SFM=cell(1,num_series);
for index =count+1:nScanTimes 
      BinName = Scans{index}; BinPath = strcat(ScanDir,BinName);
      curPtcloud = Bin2Ptcloud(BinPath);
      curr_SFM= Generate_descriptor(curPtcloud, num_sectors,num_rings,max_range);
      SFM{data_idx}=curr_SFM;
      data_idx=data_idx+1;

        % display processing
       if(rem(data_idx, 100) == 0)
        message = strcat(num2str(data_idx), " / ", num2str(num_series));
        disp(message); 
       end
        
end
%% save the log 
data_save_path = fullfile('./data/'); 
if ~exist(data_save_path,'dir')
    mkdir(data_save_path);
    
    filename = strcat(data_save_path, 'SFM', '.mat');
    save(filename, 'SFM');
    
    filename = strcat(data_save_path, 'GTposes', '.mat');
    save(filename, 'GTposes');
 else
    SFM_file = strcat(data_save_path, 'SFM',  '.mat');
    load(SFM_file);
    
    GTpose_file = strcat(data_save_path, 'GTposes', '.mat');
    load(GTpose_file);
    
    disp('- successfully loaded.');
end 





