function vfilt = filter_v(vEstimated,datasetNum)
    vfilt = zeros(size(vEstimated));
    [sampledData,~, ~] = init(datasetNum);
    timedata = vertcat(sampledData(:).t);
    ts  = timedata(2) - timedata(1);
    fs = 1/ts;
    if datasetNum == 1
        cutoff = [0.2;0.2;0.2;1.5;0.5;1.3];
        for i = 1:6
            vc = vEstimated(i,:);
            cutoff_vc = cutoff(i)/fs/2;
            order = 20;
            h = fir1(order,cutoff_vc);
            vfilt(i,:) = filter(h,1,vc);
        end
    end 
    if datasetNum == 4
        cutoff = [1.83;0.9149;1.483;0.6535;1.046;1.176];
        for i = 1:6
            vc = vEstimated(i,:);
            cutoff_vc = cutoff(i)/fs/2;
            order = 10;
            h = fir1(order,cutoff_vc);
            vfilt(i,:) = filter(h,1,vc);
        end
    end 
    
end
    