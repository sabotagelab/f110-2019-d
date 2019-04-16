import math
import numpy as np
def processGaps(gaps):
    #find linear distances
    
    maxGap = max(gaps, key = lambda gap : gap[1][0])
    linearDistances = list(map(processGap, gaps))

    return linearDistances, maxGap



def processGap(gap):
    #find total angle
    # --------|     LOW    |  |   CENTER   |  |    HIGH    |------------
    # gap = [ (range, angle), (range, angle), (range, angle)]
    #law of cosines (no sqr root)
    # a^2 = b^2 + c^2 - 2bc cosA

    totalAngle = gap[2][1] - gap[0][1]

    a2 = np.sqr(gap[0][0]) + np.sqr(gap[2][0]) - 2 * gap[0][0] * gap[2][0] * np.cos(totalAngle)

    return a2