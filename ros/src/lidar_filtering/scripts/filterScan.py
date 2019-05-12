#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class Interface:
    def __init__(self):
        rospy.init_node("lidar_filtering_node", anonymous=True)

        rospy.Subscriber("scan", LaserScan, self.filterScanData)
        self.filterScanPub = rospy.Publisher ("filter_scan", LaserScan, queue_size = 1)

    def start(self):
        rospy.spin()

    def filterScanData(self, data):
        self.lidarScan = data
        newRanges = self.filterRanges()

        msg = LaserScan()
        msg.header = data.header
        msg.angle_min = data.angle_min
        msg.angle_max = data.angle_max
        msg.angle_increment = data.angle_increment
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        msg.range_min = data.range_min
        msg.range_max = data.range_max
        msg.ranges = newRanges
        msg.intensities = data.intensities
        self.filterScanPub.publish(msg)

    def filterRanges(self, coe=1.1):
      from scipy.signal import savgol_filter
      data = np.array(self.lidarScan.ranges)
      data[np.isinf(data)] = self.lidarScan.range_max * coe
      data[np.where(data <= self.lidarScan.range_min)] = np.nan
      #data[np.isnan(data)] = lidarMessage.range_max * coe

      nanidx = np.where(np.isnan(data))[0]
      if len(nanidx):
          nanchunks = []
          last = nanidx[0]
          first = nanidx[0]
          size = 1
          for ri in xrange(1, len(nanidx)):
              if nanidx[ri] - last != 1:
                  nanchunks.append((first, size))
                  first = nanidx[ri]
                  last = first
                  size = 1
              else:
                  size += 1
                  last = nanidx[ri]

          if last != None:
              nanchunks.append((first, size))

          chunkStart = 0
          for c in nanchunks:
              if c[0] + c[1] >= len(data):
                top = .1
              else:
                top = data[c[0] + c[1]]
              if c[0] <= 0:
                bot = .1
              else:
                bot = data[c[0]-1]
              inc = (bot-top) / c[1]
              for i in xrange(chunkStart, chunkStart+c[1]):
                  data[nanidx[i]] = data[c[0]-1] + (i-chunkStart) * inc
              chunkStart += c[1]

      data = savgol_filter(data.tolist(), 11, 3)
      return data

if __name__ == "__main__":
    iface = Interface()
    iface.start()
