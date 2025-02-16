function [SFM, GTposes] = Load_Data(dim,GTpose_dir,Max_range)
%%
global data_path;
data_save_path = fullfile('./data/'); 
%%
if ~exist(data_save_path,'dir')
    % make 
    [SFM, GTposes] = Initialize_data(data_path,GTpose_dir,dim,Max_range);    
    mkdir(data_save_path);

    SFM_file = strcat(data_save_path, 'SFM', '.mat');
    save(SFM_file, 'SFM');

    GTpose_file = strcat(data_save_path, 'GTposes', '.mat');
    save(GTpose_file, 'GTposes');

else
    SFM_file = strcat(data_save_path, 'SFM',  '.mat');
    load(SFM_file);
    
    GTpose_file = strcat(data_save_path, 'GTposes', '.mat');
    load(GTpose_file);
    
    disp('- successfully loaded.');
end
end

