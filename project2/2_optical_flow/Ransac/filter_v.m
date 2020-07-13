function vfilt = filter_v(vEstimated,datasetNum)
    vfilt = zeros(size(vEstimated));
    [sampledData,~, ~] = init(datasetNum);
    timedata = vertcat(sampledData(:).t);
    ts  = timedata(2) - timedata(1);
    fs = 1/ts;
    if datasetNum == 1
        cutoff = [0.3912;0.5228;0.4574;1.5;0.5;1.3];
        for i = 1:6
            vc = vEstimated(i,:);
            cutoff_vc = cutoff(i)/fs/2;
            order = 20;
            h = fir1(order,cutoff_vc);
            vfilt(i,:) = filter(h,1,vc);
        end
    end 
    if datasetNum == 4
        cutoff = [0.5741;0.246;1.493;0.4101;0.6561;0.9841];
        for i = 1:6
            vc = vEstimated(i,:);
            cutoff_vc = cutoff(i)/fs/2;
            order = 15;
            h = fir1(order,cutoff_vc);
            vfilt(i,:) = filter(h,1,vc);
        end
    end 
    
end
    