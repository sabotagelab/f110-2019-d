import math
import numpy as np
import numpy.ma as ma

def processGaps(gaps):
    #find linear distances
    
    carWidth = .5
    minimumWidthRatio = 2
    linearDistances = np.asarray(map(processGap, gaps))
    #maxGap = max(gaps, key=lambda gap : np.average(gap))
    depths = np.array([np.average(g) for g in gaps])
    wideEnough = np.where(linearDistances > pow(carWidth * minimumWidthRatio, 2))
    mask = np.ones(len(depths), dtype=int)
    mask[wideEnough] = 0

    maskedDepths = ma.masked_array(depths, mask=mask)
    maxGap = gaps[maskedDepths.argmax()]
    



    return linearDistances, maxGap

    #deprecated for use with Kmeans 
    #maxGap = max(gaps, key = lambda gap : gap[1][0])
    #linearDistances = list(map(processGap, gaps))


#def processGap(gap):
    #return linearDistance(gap[0], gap[2])

def processGap(gap):
    return linearDistance(gap[0], gap[-1])

#find linear distance between two points in polar coords
#p<#> = (range, angle)
def linearDistance(p1, p2):
    #find total angle
    # --------|     LOW    |  |   CENTER   |  |    HIGH    |------------
    # gap = [ (range, angle), (range, angle), (range, angle)]
    #law of cosines (no sqr root)
    # a^2 = b^2 + c^2 - 2bc cosA
    totalAngle = p1[1] - p2[1]
    a2 = np.power(p1[0], 2) + np.power(p2[0], 2) - (2 * p1[0] * p2[0] * np.cos(totalAngle))
    return a2

def filterRanges(lidarMessage, coe=1.1):
    from scipy.signal import savgol_filter
    data = np.array(lidarMessage.ranges)
    data[np.isinf(data)] = lidarMessage.range_max * coe
    #data[np.isnan(data)] = lidarMessage.range_max * coe

    nanidx = np.where(np.isnan(data))[0]
    if len(nanidx):
        nanchunks = []
        last = nanidx[0]
        size = 1
        for ri in xrange(1, len(nanidx)):
            if nanidx[ri] - last != 1:
                nanchunks.append((last, size))
                last = nanidx[ri].tolist()
                size = 1
            else:
                size += 1
        if last != None:
            nanchunks.append((last, size))

    
        chunkStart = 0
        for c in nanchunks:
            inc = (data[c[0]-1] - data[c[0]+c[1]]) / c[1]
            for i in xrange(chunkStart, chunkStart+c[1]):
                data[nanidx[i]] = data[c[0]-1] + (i-chunkStart) * inc
            chunkStart += c[1]

    data = savgol_filter(data.tolist(), 11, 3)
    return data 