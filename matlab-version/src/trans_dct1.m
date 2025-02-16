function result=trans_dct1(des,factor,row)
  y = [ des(1:2:row,:); des(row:-2:2,:) ];
  b = factor .* fft(y);
  result=real(b);
end