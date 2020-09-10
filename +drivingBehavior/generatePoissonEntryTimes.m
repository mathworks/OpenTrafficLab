function entryTimes = generatePoissonEntryTimes(tMin,tMax,mu)
%generatePoissonEntryTimes generates a monotonically increassing vector of
%entry times of vehicles arriving with a rate mu (in vehicles per hour) between times tMin and tMax.
%The minHeadway guarantees that vehicles have a certain minimum time
%headway in seconds.

% Copyright 2020 - 2020 The MathWorks, Inc.

t=tMin;
minHeadway = 1;
entryTimes = [];
while t<tMax
    headway = (-log(rand)*3600/mu);
    headway = max(minHeadway,headway);
    t = t+headway;
    if t<tMax
        entryTimes(end+1)=t;
    end
end

end

