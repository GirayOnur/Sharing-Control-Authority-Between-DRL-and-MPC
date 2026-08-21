function [noisy_demand] = calc_noisy_demands(o_ind,c_ind,base_demand)
% Builds a noisy demand profile from a nominal one, for scenarios 2 and 4.
% Zero-mean Gaussian noise is added with an origin- and class-specific
% standard deviation, (200,50,40,10,40,10) over (O1,O2,O3) x (class 1,
% class 2), and the result is smoothed with a third-order low-pass
% Butterworth filter with a normalized cutoff frequency of 0.1.

scale_o1c1 = 200;
scale_o1c2 = 50;
scale_o2c1 = 40;
scale_o2c2 = 10;
scale_o3c1 = 40;
scale_o3c2 = 10;

[b,a] = butter(3,0.1);

if strcmp(o_ind,'o1')
    if strcmp(c_ind,'c1')
        noisy_demand = filtfilt(b,a, base_demand + normrnd(0,scale_o1c1,size(base_demand)));        
    elseif strcmp(c_ind,'c2')
        noisy_demand = filtfilt(b,a, base_demand + normrnd(0,scale_o1c2,size(base_demand))); 
    else
        error('c index is not valid')
    end

elseif strcmp(o_ind,'o2')
    if strcmp(c_ind,'c1')
        noisy_demand = filtfilt(b,a, base_demand + normrnd(0,scale_o2c1,size(base_demand)));  
    elseif strcmp(c_ind,'c2')
        noisy_demand = filtfilt(b,a, base_demand + normrnd(0,scale_o2c2,size(base_demand)));  
    else
        error('c index is not valid')
    end
elseif strcmp(o_ind,'o3')
    if strcmp(c_ind,'c1')
        noisy_demand = filtfilt(b,a, base_demand + normrnd(0,scale_o3c1,size(base_demand)));  
    elseif strcmp(c_ind,'c2')
        noisy_demand = filtfilt(b,a, base_demand + normrnd(0,scale_o3c2,size(base_demand)));  
    else
        error('c index is not valid')
    end
else
    error('o index is not valid')
end

end