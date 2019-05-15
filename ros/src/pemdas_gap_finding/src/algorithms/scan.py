#! /usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import math
#import matplotlib.pyplot as plt
#import seaborn as sns; sns.set()

#from pemdas_gap_finding.msg import Gap

def findGaps(msg, k=5):
    from sklearn.cluster import KMeans, DBSCAN
    from sklearn.preprocessing import StandardScaler
    return dbscanFind(msg)
    #make a list to hold data with x = range, y = angle

def kmeansFind(msg, k=5):
    X = getData(msg)
    # print('Running KMeans.....')
    kmeans = KMeans(n_clusters = k, init='random').fit(X)
    centers = kmeans.cluster_centers_
    # print('Centers found:')
    # print(centers)

    #y_kmeans = kmeans.predict(X)
    #plt.scatter(X[:, 0], X[:, 1], s=50)
    #plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
    #plt.show()

    #print(X[np.where(kmeans.labels_ == 0)])
    gaps = []
    for i in range(k):
        gaps.append([ X[np.where(kmeans.labels_ == i)][0, :].tolist(), centers[i].tolist(), X[np.where(kmeans.labels_ == i)][-1, :].tolist()])

    return gaps

def dbscanFind(msg):
    # centers = dbscan.cluster_centers_
    # print(centers)
    #plt.scatter(X[:, 0], X[:, 1], c=y_pred, cmap='Paired')
    # plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
    #plt.show()
    X = getData(msg)
    X = StandardScaler().fit_transform(X)
    db = DBSCAN(eps = 0.01*msg.range_max, min_samples=10)
    dbscan = db.fit(X)
    y_pred = db.fit_predict(X)
    gaps = []
    for i in xrange(len(set(dbscan.labels_))-1):
        gaps.append(X[np.where(dbscan.labels_ == i)].tolist())

    return gaps

def getData(msg):
    data = []
    for i in range(len(msg.ranges)):
        data.append([msg.ranges[i], ((len(msg.ranges)-1-i)*msg.angle_increment)])
        #print(msg.ranges[i], (i*0.25))

    data = np.array(data)
    #replace inf values in data with a value just above max range
    data[np.isinf(data)] = msg.range_max * 1.02
    data[np.isnan(data)] = msg.range_max * 1.02
    #data = np.nan_to_num(data)
    return(data)

def gradientScan_np(scan, z=5, coe=1.1):
    jerk = np.absolute(np.diff(np.diff(scan.ranges)))

    mean = np.mean(jerk)
    std = np.std(jerk)
    threshhold = mean + z * std

    hits = np.where(jerk > threshhold)[0] #where second gradient is > threshold

    innerClusterDiff = 1
    icd = innerClusterDiff
    # spikes = hits[np.where(np.asarray([icd+1] + np.diff(hits).tolist()) > icd)[0]].tolist()
    spikes = hits[np.where(np.asarray([icd+1] + np.diff(hits).tolist()) > icd)].tolist()
    spikes.append(len(scan.ranges)-1)

    gaps = []
    angles = [scan.angle_increment*i for i in xrange(len(scan.ranges))]
    lastSpike = 0
    for spike in spikes:
        gaps.append(zip(scan.ranges[lastSpike:spike], angles[lastSpike:spike]))
        lastSpike = spike

    return gaps

def gradientScan_nan(scan):
    data = np.array(scan.ranges)

    data[np.where(np.isnan(data))] = -5
    data[np.where(data < scan.range_min)] = -5
    data[np.where(data > scan.range_max)] = -5
    data[np.where(data == -5)] = np.nan
    chunks = []
    chunkStart = None
    chunkLen = 0
    for rdx in xrange(len(data)):
        if np.isnan(data[rdx]):
            if chunkStart == None:
                chunkStart = rdx
            chunkLen += 1
        else:
            if chunkLen > 0:
                chunks.append((chunkStart, chunkLen))
            chunkStart = None
            chunkLen = 0
    if chunkLen > 0:
        chunks.append((chunkStart, chunkLen))
    
    #data[np.where(np.isnan(data))] = 5
    data = interpolateNan(data, scan, 1.1)
    gaps = []
    angles = [scan.angle_increment*i for i in xrange(len(scan.ranges))]
    for chunk in chunks:
        gaps.append(zip(data[chunk[0]:chunk[0]+chunk[1]], angles[chunk[0]:chunk[0]+chunk[1]]))

    #print(gaps)
    return gaps


def interpolateNan(data, scan, coe):
    nanidx = np.where(np.isnan(data))[0]
    if len(nanidx):
        nanchunks = []
        chunkStart = None
        chunkLen = 0
        for rdx in xrange(len(data)):
            if np.isnan(data[rdx]):
                if chunkStart == None:
                    chunkStart = rdx
                chunkLen += 1
            else:
                if chunkLen > 0:
                    nanchunks.append((chunkStart, chunkLen))
                chunkStart = None
                chunkLen = 0
        if chunkLen > 0:
            nanchunks.append((chunkStart, chunkLen))

        chunkStart = 0
        oneSide = False
        for c in nanchunks:
            if c[0] + c[1] >= len(data):
                top = data[c[0]-2]
                bot = data[c[0]-1]
                oneSide = True
            elif c[0] <= 0:
                top = data[c[0] + c[1]]
                bot = data[c[0] + c[1] + 1]
                oneSide = True
            else:
                top = data[c[0] + c[1]]
                bot = data[c[0]-1]
            #if we are using a one-sided gradient to interpolate, dont divide
            inc = (bot-top) / (c[1]+1) if not oneSide else 1
            for i in xrange(c[0], c[0]+c[1]):
                offset = (i-c[0]) * inc
                data[i] = min(bot + offset, scan.range_max * coe)

    return data

