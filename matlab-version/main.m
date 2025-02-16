clear; clc;
addpath(genpath('src'));
global data_path; 
%  directory structure 
% - 00
%   - 00.csv (gtpose)
%   - velodyne
%      - <00001.bin>
%      - <00002.bin>
%% data path setting
data_path = '../../loop_closure/08/'; 
GTposes_file='08.csv';

%% loading ground-truth poses and creating structural feature matrix SFM
SFM_dim = [80,120];Range=80; 
[SFM, GT_poses] = Load_Data(SFM_dim,GTposes_file,Range);

%% loop parameter setting
 revisit_thres = 4; % 
 num_candidates = 300;
 
%% BIF parameter setting
bif_size=16;%BIF size
num_queries = length(GT_poses);
BIF=cell(1,num_queries);
dct_result=cell(1,num_queries);
rim=zeros(num_queries,SFM_dim(2));
ringkeys=zeros(num_queries,SFM_dim(1));
BIF_loop=zeros(num_queries-num_candidates,3);
%% DCT transform setting
% You can use matlab's dct2 function directly, but it is less efficient
%one dimension dct transform  needs factor
 first_factor = 2*exp(-1i*(0:SFM_dim(1)-1)'*pi/(2*SFM_dim(1)))/sqrt(2*SFM_dim(1));
 first_factor(1) = first_factor(1) / sqrt(2);
 trans_factor_1 = first_factor(:,ones(1,SFM_dim(2)));
 
 %two dimension dct transform  needs factor
 sencond_factor = 2*exp(-1i*(0:SFM_dim(2)-1)'*pi/(2*SFM_dim(2)))/sqrt(2*SFM_dim(2));
 sencond_factor(1) = sencond_factor(1) / sqrt(2);
 trans_factor_2 = sencond_factor(:,ones(1,SFM_dim(1)));
 trans_factor_22= trans_factor_2(1:bif_size,1:bif_size);
 
 %%  create useful informatin about BIF
 for index=1:num_queries
     % create bif by dct2;
    current_sfm=SFM{index};
    imgdct1=trans_dct1(current_sfm,trans_factor_1,80).';
    dct_result{index}=imgdct1(:,1:bif_size);
    imgdct2=trans_dct2(imgdct1(:,1:bif_size),trans_factor_22);
    bif_mean=sum(imgdct2(:))/(bif_size*bif_size);  
    current_bif=(imgdct2>=bif_mean);
    BIF{index}=current_bif;
    % create view_rim align vector
    rim(index,:)=view_rim(current_sfm);
    % create  ringkey vector
    ringkeys(index,:)=ringkey(current_sfm);
 end
 %% search the best similar frame by our method
 disp('-----------------------------------------------------');
 disp('Place recognition task begins');
 for query_idx = 1:num_queries     
    query_pose = GT_poses(query_idx,:);
    query_bif=BIF{query_idx};
    query_rim=rim(query_idx,:);
    query_rk=ringkeys(query_idx,:);
    min_hamming_dis=1;

    if( query_idx <= num_candidates )
       continue;
    end

        % judging revisitness by using ground-truth poses
       is_revisit= Loop_truth(query_pose, GT_poses(1:query_idx-num_candidates, :), revisit_thres);
    
        % k-nearest neighbor search
        tree = createns(ringkeys(1:query_idx-num_candidates, :), 'NSMethod', 'kdtree'); 
        candidates = knnsearch(tree, query_rk, 'K', 20); %10 is ok
  
    for ith_candidate = 1:length(candidates)
        candidate_node_idx = candidates(ith_candidate);
        angle=align_rim(query_rim,rim(candidate_node_idx,:));
        
        % yaw=0,BIF cannot change
         if angle==0
        hamming=calculate_hamming_dis(query_bif,BIF{candidate_node_idx});
        else
         align_bif=calculate_align_bif(dct_result{candidate_node_idx},trans_factor_22,angle);
         hamming=calculate_hamming_dis(query_bif,align_bif);
         end
        if hamming<min_hamming_dis
        min_hamming_dis=hamming;
        near_idx=candidate_node_idx;
        end
    end  
      BIF_loop(query_idx-num_candidates,:)=[near_idx,is_revisit,min_hamming_dis]; 
      
      if( rem(query_idx, 100) == 0)
        disp( strcat(num2str(query_idx/num_queries * 100), ' % processed') );
      end
    
 end
%% visiualize GT path
figure(1);hold on;
plot(GT_poses(:,1), GT_poses(:,2),'LineWidth',2);
axis equal; grid on;
legend('Groud-Truth');

%%  save the loop information
    data_save_path = fullfile('./data/'); 
    filename = strcat(data_save_path, 'BIF_loop', '.mat');
    save(filename, 'BIF_loop');
