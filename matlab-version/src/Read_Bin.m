function ptcloud = Read_Bin(bin_path)
%% Read a frame
fid = fopen(bin_path, 'rb'); raw_data = fread(fid, [4 inf], 'single'); fclose(fid);
points = raw_data(1:3,:)';
ptcloud = pointCloud(points);
end % end of function
