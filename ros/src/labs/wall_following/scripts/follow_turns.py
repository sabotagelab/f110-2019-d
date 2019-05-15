#!/usr/bin/env python
import rospy
import numpy as np
import math
import Queue as queue
from wall_following.msg import follow_type
from pemdas_gap_finding.msg import Gaps
import os, sys
import yaml

DO_VISUALIZATION = True
if DO_VISUALIZATION:
    from visualization_msgs.msg import Marker, MarkerArray
    from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
    from std_msgs.msg import Header

class Interface:
    def __init__(self):
        rospy.init_node("follow_turns_node", anonymous=True)

        self.gapSub = rospy.Subscriber("lidar_gaps", Gaps, self.determineInstruction)
        self.followPub = rospy.Publisher("follow_type", follow_type, queue_size=5)
        self.followPubRviz = rospy.Publisher("follow_type_rviz", Marker, queue_size=5)

        #visualization topics
        if DO_VISUALIZATION:
            self.goodGapVisualizationPub = rospy.Publisher("visualize_good_gaps", MarkerArray, queue_size=10)

        self.instructions = {
            "C" : 0,    #followcenter
            "R" : 1,    #followright
            "L" : 2,    #followleft
            "S" : 3,    #follow straight (gap)
            "I" : 3
        }

        #standard deviations to use for deciding if a gap is "significant"
        self.goodGapThresholdFactor = 2

        #how many "significant" gaps will trigger advance to next instruction
        self.newInstructionGapCountThresh = 1
        self.forwardHeading = np.deg2rad(135)

        self.defaultInstruction= "C"
        self.instructionDir = "../explicit_instructions/"
        self.instructionFile = "levine_instructions.dat"

        self.loadConfig()

        self.defaultInstructionEnum = self.instructions[self.defaultInstruction]
        self.currentInstruction = None
        self.currentInstructionEnum = None

        self.inTurnNow = False
        self.followGapAngle = 0
        self.maxMarkers = 0

        self.instructionQueue = queue.Queue()


        self.instructionFile = os.path.join(os.path.dirname(__file__), (self.instructionDir + self.instructionFile))
        print("loading explicit instructions from {}".format(self.instructionFile))
        self.loadInstructions(self.instructionFile)

    def loadConfig(self):
        configFile = "config.yaml"
        dirname = os.path.dirname(__file__)
        filepath = os.path.join(dirname, '../config/' + configFile)

        with open (filepath, 'r') as f:
            doc = yaml.load(f)
            doc = doc["instruction"]

        if "goodGapThresh" in doc:
            self.goodGapThresholdFactor = doc["goodGapThresh"]
        if "newInstructionGapCount" in doc:
            self.newInstructionGapCountThresh = doc["newInstructionGapCount"]
        if "defaultInstruction" in doc:
            self.defaultInstruction = doc["defaultInstruction"]
        if "instructionDirectory" in doc:
            self.instructionDir = doc["instructionDirectory"]
        if "instructionFile" in doc:
            self.instructionFile = doc["instructionFile"]

    def start(self):
        rospy.spin()

    def loadInstructions(self, file):
        with open(file, "r") as instructionData:
            instructionString = instructionData.readline()
            count = 1
            for inst in instructionString:
                self.instructionQueue.put(inst)
                print("Loaded instruction {}: {}".format(count, inst))
                count += 1

    def determineInstruction(self, data):
        if self.countGoodGaps(data) > self.newInstructionGapCountThresh:
            self.inTurnNow = True
            if DO_VISUALIZATION:
                self.visualizeTurns(self.inTurnNow)
            if not self.instructionQueue.empty() and self.currentInstruction == None:
                self.currentInstruction = self.instructionQueue.get()
                self.currentInstructionEnum = self.instructions[self.currentInstruction]
            else:
                print("Instruction Queue empty, follow_turns executing default instruction...")
                self.currentInstruction = self.defaultInstruction
            self.follow(self.currentInstruction)
            if self.currentInstructionEnum == 3: #only if following gap
                self.followGapAngle = self.findHeadingGapAngle(data.gaps)
        else:
            self.inTurnNow = False
            self.follow(self.defaultInstruction)
            self.currentInstruction = None

    def visualizeTurns(self, text):
        marker = Marker()
        marker.type = marker.TEXT_VIEW_FACING
        marker.id = 0
        marker.lifetime = rospy.Duration(2)
        marker.pose= Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1))
        marker.scale = Vector3(0.06, 0.06, 0.06)
        marker.header = Header( frame_id='base_link')
        marker.color.a = 1
        marker.text = str(text)
        self.followPubRviz.publish(marker)


    def countGoodGaps(self, gaps):
        mean = np.mean(gaps.scores)
        std = np.std(gaps.scores)
        thresh = mean + (std * self.goodGapThresholdFactor)

        criticalGapIndices = np.argwhere(gaps.scores > thresh)

        if DO_VISUALIZATION:
            goodColor = [.1, 1.0, .1]
            badColor = [1.0, 0, 0]
            gapMarkers = MarkerArray()
            gapMarkers.markers = []
            self.maxMarkers = max(len(gaps.gaps), self.maxMarkers)
            for gdx in xrange(self.maxMarkers):
                marker = Marker()
                if gdx >= len(gaps.gaps):
                    marker.id = 100 + gdx
                    marker.action = marker.DELETE
                    gapMarkers.markers.append(marker)
                    continue

                gap = gaps.gaps[gdx]
                goodGap = gdx in criticalGapIndices
                color = goodColor if goodGap else badColor

                # Specify the frame in which to interpret the x,y,z coordinates. It is the laser frame.
                marker.id = 100 + gdx
                marker.header.frame_id = "/laser"
                marker.pose.position.x = gap.points[1].range * np.cos(gap.points[1].angle - np.deg2rad(135))
                marker.pose.position.y = gap.points[1].range * np.sin(gap.points[1].angle - np.deg2rad(135))
                marker.pose.position.z = 0 # or set this to 0

                marker.type = marker.SPHERE
                marker.action = marker.MODIFY
                marker.lifetime = rospy.rostime.Duration(secs=.2)

                marker.scale.x = 1.0 - (not goodGap) * .5
                marker.scale.y = 1.0 - (not goodGap) * .5
                marker.scale.z = 1.0 - (not goodGap) * .5
                marker.color.r = color[0]
                marker.color.g = color[1]
                marker.color.b = color[2]
                marker.color.a = 1.0

                gapMarkers.markers.append(marker)
            rospy.loginfo("Published good gap markers")
            self.goodGapVisualizationPub.publish(gapMarkers)

        return len(criticalGapIndices)

    #the center angle of the gap with heading closest to 135 (lidar forward)
    def findHeadingGapAngle(self, gaps):
        centerAngle = lambda g : g.points[1].angle

        bestGap = None
        bestError = 2*math.pi #cannot be off by a full circle!!!
        for gap in gaps[1:]:
            error = abs(centerAngle(gap) - self.forwardHeading)
            if error < bestError:
                bestError = error
                bestGap = gap

        return centerAngle(bestGap)

    def follow(self, instruction):
        msg = follow_type()
        msg.type = self.instructions[instruction]
        msg.gap_angle = self.followGapAngle
        msg.turnMode = self.inTurnNow
        self.followPub.publish(msg)


if __name__ == "__main__":
    iface = Interface()
    iface.start()
