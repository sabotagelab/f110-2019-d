#!/usr/bin/env python
import rospy
import numpy as np
import math
import Queue as queue
from wall_following.msg import follow_type
from pemdas_gap_finding.msg import Gaps
import os, sys
import yaml

class Interface:
    def __init__(self):
        rospy.init_node("follow_turns_node", anonymous=True)

        self.gapSub = rospy.Subscriber("lidar_gaps", Gaps, self.determineInstruction)
        self.followPub = rospy.Publisher("follow_type", follow_type, queue_size=5)

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

        self.instructionQueue = queue.Queue()


        self.instructionFile = os.path.join(os.path.dirname(__file__), self.instructionDir + self.instructionFile)
        self.loadInstructions(self.instructionFile)

    def loadConfig(self):
        configFile = "config.yaml"
        if len(sys.argv) > 1:
            configFile = sys.argv[1]
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
            instructionString = instructionData.readLine()
            count = 1
            for inst in instructionString:
                self.instructionQueue.put(inst)
                print("Loaded instruction {}: {}".format(count, inst))
                count += 1

    def determineInstruction(self, data):
        if self.countGoodGaps(data.gaps) > self.newInstructionGapCountThresh:
            if self.currentInstruction == None:
                self.currentInstruction = self.instructionQueue.get()
                self.currentInstructionEnum = self.instructions[self.currentInstruction]
            self.follow(self.currentInstruction)
            if self.currentInstructionEnum == 3: #only if following gap
                self.followGapAngle = self.findHeadingGapAngle(data.gaps)
        else:
            self.follow(self.defaultInstruction)
            self.currentInstruction = None


    def countGoodGaps(self, gaps):
        mean = np.mean(gaps.scores)
        std = np.std(gaps.scores)
        thresh = mean + (std * self.goodGapThresholdFactor)

        criticalGaps = np.argwhere(gaps.scores > thresh)

        return len(criticalGaps)

    #the center angle of the gap with heading closest to 135 (lidar forward)
    def findHeadingGapAngle(self, gaps):
        centerAngle = lambda g : g.features[len(g.features)//2].angle

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
        self.followPub.publish(msg)


if __name__ == "__main__":
    iface = Interface()
    iface.start()
