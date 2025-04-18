function WriteBin(dataArray, binName, path)

fid = fopen(fullfile(path, binName), 'wb');
fwrite(fid, int16(dataArray), 'int16'); % 384*samples

fclose(fid);

end