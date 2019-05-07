import rospy
import queue

class Interface:
    def __init__(self, instructionFile):
        rospy.init_node("follow_turns", anonymous=True)

        self.gapSub = rospy.Subscriber("all gaps", gaps, self.determineInstruction)
        self.followPub = rospy.Publisher("follow_type", follow_type)

        self.instructions = {
            "center" : 0,
            "right" : 1,
            "left" : 2,
            "gap" : 3,
            "ignore" : 3
        }

        self.instructionQueue = queue()
        self.loadInstructions(instructionFile)
    
    def loadInstructions(self, file):
        with open(file, "")

    def determineInstruction(self, data):
        if countGoodGaps > self.newInstructionGapCountThresh:
            if self.currentInstruction != None:
                self.currentInstruction = instructionQueue.pop()

    
    def countGoodGaps(self, gaps):


    def findHeadingGap(self, gaps):
        return #the gap with heading closest to 135

    def follow(self, instruction):
        msg = follow_types()
        msg.type = self.instructions[instructions]
        msg.gap_angle = self.followGapAngle
        self.followPub.publish(msg)



    