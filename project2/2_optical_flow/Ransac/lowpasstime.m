function sTime = lowpasstime(n)
    datasetNum = n;

    [sampledData,~, ~] = init(datasetNum);
    timedata = vertcat(sampledData(:).t);
    ts  = timedata(2) - timedata(1);
    fs = 1/ts;
    % were used to find the cut off freuency
    %nfft = length(timedata);
    % nfft2 = 2.^nextpow2(nfft);
    %fy = fft(timedata,nfft2);
    %fy = fy(1:nfft2/2);
    %xfft = fs.*(0:nfft2/2-1)/nfft2;
    %plot(xfft,abs(fy/max(fy)))
    cutoff = 5/fs/2;
    order = 10;
    h = fir1(order,cutoff);
    sTime = filter(h,1,timedata);
   % disp(size(sTime))
end 
