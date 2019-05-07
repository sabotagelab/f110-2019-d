import rospy
import queue
import numpy as np
from wall_following.msg import follow_type
from pemdas_gap_finding.msg import Gaps

class Interface:
    def __init__(self, instructionFile):
        rospy.init_node("follow_turns", anonymous=True)

        self.gapSub = rospy.Subscriber("all gaps", Gaps, self.determineInstruction)
        self.followPub = rospy.Publisher("follow_type", follow_type)

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

        self.defaultInstruction= "C"
        self.currentInstruction = None

        self.instructionQueue = queue.Queue()
        self.loadInstructions(instructionFile)
    
    def start(self):
        rospy.spin()

    def loadInstructions(self, file):
        with open(file, "r") as instructionData:
            instructionString = instructionData.readLine()
            for inst in instructionString:
                self.instructionQueue.put(inst)

    def determineInstruction(self, data):
        if self.countGoodGaps(data.gaps) > self.newInstructionGapCountThresh:
            if self.currentInstruction != None:
                self.currentInstruction = self.instructionQueue.get()
            self.follow()
        else:
            self.follow(self.defaultInstruction)
            self.currentInstruction = None

    
    def countGoodGaps(self, gaps):
        mean = np.mean(gaps.scores)
        std = np.std(gaps.scores)
        thresh = mean + (std * self.goodGapThesholdFactor)

        criticalGaps = np.argwhere(gaps.scores > thresh)

        return len(criticalGaps)

    def findHeadingGap(self, gaps):
        return #the gap with heading closest to 135

    def follow(self, instruction=self.currentInstruction):
        msg = follow_type()
        msg.type = self.instructions[instruction]
        msg.gap_angle = self.followGapAngle
        self.followPub.publish(msg)


if __name__ == "__main__":
    iface = Interface("config/levineInstruction.dat")
    iface.start()