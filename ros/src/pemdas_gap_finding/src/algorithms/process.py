import math
import numpy as np
import numpy.ma as ma

def processGaps(gaps):
    #find linear distances
    
    carWidth = .5
    minimumWidthRatio = 2
    linearDistances = np.asarray(map(processGap, gaps))
    #maxGap = max(gaps, key=lambda gap : np.average(gap))
    scores = np.array([np.average(g) for g in gaps])
    scores -= np.min(scores)
    scores *= linearDistances
    wideEnough = np.where(linearDistances > pow(carWidth * minimumWidthRatio, 2))
    mask = np.ones(len(scores), dtype=int)
    mask[wideEnough] = 0

    maskedDepths = ma.masked_array(scores, mask=mask)
    maxGap = gaps[maskedDepths.argmax()]


    return scores, linearDistances, maxGap

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
