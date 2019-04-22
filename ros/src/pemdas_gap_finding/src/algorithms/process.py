import math
import numpy as np

def processGaps(gaps):
    #find linear distances

    maxGap = max(gaps, key = lambda gap : np.mean(gap))
    linearDistances = list(map(processGap, gaps))

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
    