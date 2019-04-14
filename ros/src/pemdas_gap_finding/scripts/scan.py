#! /usr/bin/env python


import rospy
from sensor_msgs.msg import LaserScan
from sklearn.cluster import KMeans
import numpy as np
import math
import matplotlib.pyplot as plt
#import seaborn as sns; sns.set()

def callback(msg):
    #make a list to hold data with x = range, y = angle
    X = getData(msg)
    kmeans = KMeans(n_clusters = 5, init='random').fit(X)
    centers = kmeans.cluster_centers_
    print(centers)
    #y_kmeans = kmeans.predict(X)
    # plt.scatter(X[:, 0], X[:, 1], s=50)
    # plt.scatter(centers[:, 0], centers[:, 1], c='black', s=200, alpha=0.5)
    # plt.show()


    # for i in range(5):
    #     print(np.where(kmeans.labels_ == i)[0, :]) #min
    #     print(np.where(kmeans.labels_ == i)[-1, :]) #max


def getData(msg):
    data = []
    for i in range(len(msg.ranges)):
        data.append([msg.ranges[i], (i*0.25)])
        #print(msg.ranges[i], (i*0.25))

    data = np.array(data)
    data[np.isinf(data)] = 100
    #data = np.nan_to_num(data)
    return(data)



rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)
rospy.spin()
