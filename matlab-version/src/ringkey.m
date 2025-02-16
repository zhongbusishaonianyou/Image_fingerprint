function  ring_key  = ringkey(sfm)
num_rings = size(sfm, 1);
ring_key = zeros(1, num_rings);
for ith=1:num_rings
    ith_ring = sfm(ith,:);
    ring_key(ith) = sum(ith_ring);
end
end

