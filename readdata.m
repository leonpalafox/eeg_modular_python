fid = fopen('/build/raw_data','r');
fseek(fid, 0, 'bof');
c = fread(fid, Inf, '*single');
m = vec2mat(c,65);
