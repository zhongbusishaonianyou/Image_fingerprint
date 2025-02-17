function Ptcloud = Bin2Ptcloud(BinPath)
fid = fopen(BinPath, 'rb'); raw_data = fread(fid, [4 inf], 'single'); fclose(fid);
points = raw_data(1:3,:)';
Ptcloud = pointCloud(points);
end 
