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

      #we are going to interpolate all nan, too large, and too small values
      data[np.isnan(data)] = -5
      data[np.where(data <= self.lidarScan.range_min)] = -5
      data[np.where(data >= self.lidarScan.range_max)] = -5
      #data[np.isnan(data)] = lidarMessage.range_max * coe

      self.interpolateNan(data, coe)
      #data[0:int(len(data)/7)] = self.lidarScan.range_min
      #data[int(len(data)*6/7):len(data)-1] = self.lidarScan.range_min
      #data = savgol_filter(data.tolist(), 11, 3)
      return data

    def interpolateNan(self, data, coe):
      nanidx = np.where(data == -5)[0]
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
              for i in xrange(chunkStart, chunkStart+c[1]):
                offset = (i-chunkStart) * inc
                linearization = np.cos((i-chunkStart) * self.lidarScan.angle_increment) * offset
                data[nanidx[i]] = min(data[c[0]-1] + offset, self.lidarScan.range_max * coe)
              chunkStart += c[1]

      return data


if __name__ == "__main__":
    iface = Interface()
    iface.start()
