function  align_bif=calculate_align_bif(des2,trans_factor,angle)
des=circshift(des2,angle,1);
imgdct2=trans_dct2(des,trans_factor);
mean2=sum(imgdct2(:))/(16*16);
align_bif=(imgdct2>=mean2);
end