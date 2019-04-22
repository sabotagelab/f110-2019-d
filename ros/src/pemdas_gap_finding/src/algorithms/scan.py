#! /usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import KMeans, DBSCAN
import numpy as np
import math
import matplotlib.pyplot as plt
#import seaborn as sns; sns.set()

#from pemdas_gap_finding.msg import Gap

def findGaps(msg, k=5):
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

def gradientScan_np(scan, z=2, coe=1.1):
    data = np.array(scan.ranges)
    data[np.isinf(data)] = scan.range_max * coe
    data[np.isnan(data)] = scan.range_max * coe

    jerk = np.absolute(np.diff(np.diff(data)))

    mean = np.mean(jerk)
    std = np.std(jerk)
    threshhold = mean + z * std

    hits = np.where(jerk > threshhold)[0] #where second gradient is > threshold

    spikes = hits[np.where(np.diff(hits) > 1)[0]].tolist()
    spikes.append(len(scan.ranges)-1) 

    gaps = []
    angles = [scan.angle_increment*i for i in xrange(len(scan.ranges), 0, -1)]
    lastSpike = 0
    for spike in spikes:
        gaps.append(zip(scan.ranges[lastSpike:spike], angles[lastSpike:spike]))
        lastSpike = spike
    
    return gaps

def gradientScan(data):
    from scipy.signal import savgol_filter
    #Set inf and nan values to a value greater than the furthest scan
    scan = savgol_filter(data.ranges,11,3)
    coe = 1.1
    scan = [(data.range_max * coe) if math.isnan(x) else x for x in scan]
    scan = [(data.range_max * coe) if math.isinf(x) else x for x in scan]
    k = 1
    dscan = []
    ddscan = []

    #Find difference between subsequent elements in scan
    for i in range(len(scan)-1):
        dscan.append(scan[i+1]-scan[i])
        if i > 0:
            ddscan.append(abs(dscan[i]-dscan[i-1])) 
    z = 2
    mean = sum(ddscan) / len(ddscan)   # mean
    var  = sum(pow(x-mean,2) for x in ddscan) / len(ddscan)  # variance
    std  = np.sqrt(var)
    thres = mean + z*std
    hits = []
    last = 0
    for i in range(len(ddscan)):
        if (ddscan[i] > thres):
            hits.append(i)
            if i-last>1:
                k = k+1
            last = i
    hits.append(len(scan)-1)
    spike = []
    feat = [0]
    for i in range(len(hits)-1):
        if hits[i+1]-hits[i]==1:
            spike.append(hits[i])
        else:
            spike.append(hits[i])
            feat.append(int(round((spike[0]+spike[-1])/2)))
            spike = []

    feat.append(len(scan)-1)

    gaps = []
    angles = [data.angle_increment*i for i in xrange(len(scan), 0, -1)]
    print(len(angles))
    print(len(scan))
    print(feat)
    for idx in xrange(len(feat)):
        print(idx)
        gap = zip(scan[feat[idx]:feat[idx+1]], angles[feat[idx]:feat[idx+1]])
        gaps.append(gap)
    return gaps

    #bestGap = [0,0]
    #for i in range(len(feat)-1):
        #newb = np.average(scan[feat[i]:feat[i+1]])
        #if newb>bestGap[0]:
            #dist = np.average(scan[feat[i]:feat[i+1]])
            #ang = (round((feat[i+1]+feat[i])/2)*data.angle_increment)
            #ang -= (data.angle_max-data.angle_min)/2
            #bestGap = [dist , ang]
    #return bestGap