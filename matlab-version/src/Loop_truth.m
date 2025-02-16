function is_revisit= Loop_truth(query_pose, history_poses, thres)
num = size(history_poses,1);
dists = zeros(1, num);
   for i=1:num
      dist = norm(query_pose - history_poses(i, :));
      dists(i) = dist;    
   end  
  min_dist = min(dists);
   if ( min_dist <= thres ) 
    is_revisit = 1;
   else
    is_revisit = 0;
  end
end

