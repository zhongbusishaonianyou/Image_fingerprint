function dis=calculate_hamming_dis(bif1,bif2)
diff=xor(bif1,bif2);    
hamming=sum(diff(:));   
dis=hamming/(16*16);
end