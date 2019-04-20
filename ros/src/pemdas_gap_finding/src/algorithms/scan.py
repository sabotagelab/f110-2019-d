#! /usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import KMeans
import numpy as np
import math
import matplotlib.pyplot as plt
#import seaborn as sns; sns.set()

#from pemdas_gap_finding.msg import Gap

def findGaps(msg, k=5):
    #make a list to hold data with x = range, y = angle
    X = getData(msg)
    # print('Running KMeans.....')
    # kmeans = KMeans(n_clusters = k, init='random').fit(X)
    # centers = kmeans.cluster_centers_
    # print('Centers found:')
    # print(centers)

    #y_kmeans = kmeans.predict(X)
    #plt.scatter(X[:, 0], X[:, 1], s=50)
    #plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
    #plt.show()

    #print(X[np.where(kmeans.labels_ == 0)])
    # gaps = []
    # for i in range(k):
    #     gaps.append([ X[np.where(dbscan.labels_ == i)][0, :].tolist(), centers[i].tolist(), X[np.where(dbscan.labels_ == i)][-1, :].tolist()])


    #DBSCAN
    db = DBSCAN(eps = 0.01*msg.range_max, min_samples=3)
    dbscan = db.fit(X)
    y_pred = db.fit_predict(X)
    # centers = dbscan.cluster_centers_
    # print(centers)
    plt.scatter(X[:, 0], X[:, 1], c=y_pred, cmap='Paired')
    # plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
    plt.show()
    gaps = []
    for i in range(len(set(dbscan.labels_))):
        gaps.append([ X[np.where(dbscan.labels_ == i)].tolist()])

    return(gaps)


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
