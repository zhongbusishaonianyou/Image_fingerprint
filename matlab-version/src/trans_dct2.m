function result=trans_dct2(des,factor)
 y = [ des(1:2:120,:); des(120:-2:2,:)];
  b =  fft(y);
  c=factor .*b(1:16,:);
  result=real(c);
end