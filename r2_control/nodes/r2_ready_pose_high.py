#! /usr/bin/env python

import roslib; roslib.load_manifest('r2_control')
import rospy
import actionlib
import math
import random

from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from copy import copy, deepcopy

TORAD = math.pi/180.0
TODEG = 1.0/TORAD

class r2ReadyPose :

    def __init__(self, joint_names, wp, controller):

        self.currentData = None
        self.desiredData = None
        self.deadlineData = None

        self.jointNames = joint_names
        self.numJoints = len(joint_names)
        self.waypoints = wp

        self.currentState = JointState()
        self.currentState.position = [0]*self.numJoints
        self.currentState.velocity = [0]*self.numJoints
        self.currentState.effort = [0]*self.numJoints

        rospy.Subscriber("r2/joint_states", JointState, self.jointStateCallback)
        self.trajPublisher = rospy.Publisher(controller + '/command', JointTrajectory)
        self.trajClient = actionlib.SimpleActionClient(controller + '/follow_joint_trajectory', FollowJointTrajectoryAction)

        self.trajClient.wait_for_server()

        self.actionGoal = FollowJointTrajectoryGoal()


    def getNumJoints(self) :
        return self.numJoints

    def jointStateCallback(self, data):
        self.currentState = data

    def computeTrajectory(self, desiredData, deadline):

        jointTraj = JointTrajectory()
        currentState = copy(self.currentState)
        desiredState = copy(desiredData)

        # create simple lists of both current and desired positions, based on provided desired names
        rospy.loginfo("r2ReadyPose::computeTrajectory() -- finding necessary joints")
        desiredPositions = []
        currentPositions = []
        for desIndex in range(len(desiredState.name)) :
            for curIndex in range(len(currentState.name)) :
                if ( desiredState.name[desIndex] == currentState.name[curIndex] ) :
                    desiredPositions.append(desiredState.position[desIndex])
                    currentPositions.append(currentState.position[curIndex])

        rospy.loginfo("r2ReadyPose::computeTrajectory() -- creating trajectory")
        jointTraj.joint_names = desiredState.name
        jointTraj.points = list()

        for j in range(self.waypoints) :
            trajPoint = JointTrajectoryPoint()

            t = (deadline / self.waypoints) * (j + 1)
            trajPoint.time_from_start = rospy.Duration(t)

            trajPoint.positions = list()
            for i in range(len(desiredPositions)) :
                trajPoint.positions.append( self.minJerk(currentPositions[i], desiredPositions[i], deadline, t) )

            jointTraj.points.append(trajPoint)

        rospy.loginfo("r2ReadyPose::moveToGoal() -- using tolerances")

        return jointTraj


    def minJerk(self, start, end, duration, t):
        tOverD = float(t) / float(duration)
        return start + (end - start)*( 10*math.pow(tOverD,3) - 15*math.pow(tOverD,4) + 6*math.pow(tOverD,5) )

    def moveToGoal(self, jointGoal, deadline, useTolerances) :

        self.actionGoal.trajectory = self.computeTrajectory(jointGoal, deadline)

        offset = 0

        if useTolerances :
            rospy.loginfo("r2ReadyPose::moveToGoal() -- using tolerances")
            self.actionGoal.path_tolerance = []
            self.actionGoal.goal_tolerance = []

            for i in range(self.numJoints):
                tol.position = 0.2
                tol.velocity = 1
                tol.acceleration = 10
                tol.name = self.jointNames[i]
                self.actionGoal.path_tolerance.append(tol)
                self.actionGoal.goal_tolerance.append(tol)

        else :
            rospy.loginfo("r2ReadyPose::moveToGoal() -- not using tolerances")

        self.actionGoal.goal_time_tolerance = rospy.Duration(10.0)

        # send goal nad monitor response
        self.trajClient.send_goal(self.actionGoal)

        rospy.loginfo("r2ReadyPose::moveToGoal() -- returned state: %s", str(self.trajClient.get_state()))
        rospy.loginfo("r2ReadyPose::moveToGoal() -- returned result: %s", str(self.trajClient.get_result()))

        return

    def formatJointStateMsg(self, j, offset) :

        if not (len(j) == self.numJoints) :
            rospy.logerr("r2ReadyPose::formatJointStateMsg() -- incorrectly sized joint message")
            return None

        js = JointState()
        js.header.seq = 0
        js.header.stamp = rospy.Time.now()
        js.header.frame_id = ""
        js.name = self.jointNames
        js.position = []

        for i in range(self.numJoints):
            js.position.append(j[i])

        return js


if __name__ == '__main__':
    rospy.init_node('r2_ready_pose')
    try:
        arm_joints = ["joint0", "joint1", "joint2", "joint3", "joint4", "wrist/pitch", "wrist/yaw"]
        hand_joints = ["thumb/roll", "thumb/proximal", "thumb/medial", "thumb/distal", "index/yaw", "index/proximal", "index/medial", "middle/yaw", "middle/proximal", "middle/medial", "ringlittle/ring", "ringlittle/ringMedial", "ringlittle/little", "ringlittle/littleMedial"]
        neck_joints = ["joint0", "joint1", "joint2"]

        leftNames = []
        rightNames = []
        for i in range(len(arm_joints)):
          leftNames.append("r2/left_arm/" + arm_joints[i])
          rightNames.append("r2/right_arm/" + arm_joints[i])

        rightHandNames = []
        leftHandNames = []
        for i in range(len(hand_joints)):
          rightHandNames.append("r2/right_arm/hand/" + hand_joints[i])
          leftHandNames.append("r2/left_arm/hand/" + hand_joints[i])

        neckNames = []
        for i in range(len(neck_joints)):
          neckNames.append("r2/neck/" + neck_joints[i])

        r2TrajectoryGeneratorLeft = r2ReadyPose(leftNames, 500, '/r2/l_arm_controller')
        r2TrajectoryGeneratorRight = r2ReadyPose(rightNames, 500, '/r2/r_arm_controller')
        r2TrajectoryGeneratorNeck = r2ReadyPose(neckNames, 500, '/r2/neck_controller')
        r2TrajectoryGeneratorLeftHand = r2ReadyPose(rightHandNames, 10, '/r2/r_hand_controller')
        r2TrajectoryGeneratorRightHand = r2ReadyPose(leftHandNames, 10, '/r2/l_hand_controller')
        rospy.sleep(2)

        lhrp = [0]*14
        rhrp = [0]*14

        lrp1 = [50.0*TORAD, -80.0*TORAD, -105.0*TORAD, -140.0*TORAD, 80.0*TORAD, 0.0*TORAD, 0.0*TORAD]
        rrp1 = [-50.0*TORAD, -80.0*TORAD, 105.0*TORAD, -140.0*TORAD, -80.0*TORAD, 0.0*TORAD, 0.0*TORAD]

        rrp2 = [ 0.4, -0.5,  1.57, -2.0, -0.7, 0.3,  0.6]
        lrp2 = [-0.4, -0.5, -1.57, -2.0,  0.7, 0.3, -0.6]

        nrp = [-20.0*TORAD, 0.0*TORAD, -15.0*TORAD]
        print "r2ReadyPose() -- moving to ready pose"

        jointGoalNeck = r2TrajectoryGeneratorNeck.formatJointStateMsg(nrp, 0)
        jointGoalLeftHand = r2TrajectoryGeneratorLeftHand.formatJointStateMsg(lhrp, 0)
        jointGoalRightHand = r2TrajectoryGeneratorRightHand.formatJointStateMsg(rhrp, 0)
        r2TrajectoryGeneratorLeftHand.moveToGoal(jointGoalLeftHand, 0.1, False)
        r2TrajectoryGeneratorRightHand.moveToGoal(jointGoalRightHand, 0.1, False)
        r2TrajectoryGeneratorNeck.moveToGoal(jointGoalNeck, 0.5, False)

        jointGoalLeft = r2TrajectoryGeneratorLeft.formatJointStateMsg(lrp1, 0)
        jointGoalRight = r2TrajectoryGeneratorRight.formatJointStateMsg(rrp1, 0)
        r2TrajectoryGeneratorLeft.moveToGoal(jointGoalLeft, 0.5, False)
        r2TrajectoryGeneratorRight.moveToGoal(jointGoalRight, 0.5, False)

        rospy.sleep(3)
        jointGoalLeft = r2TrajectoryGeneratorLeft.formatJointStateMsg(lrp2, 0)
        jointGoalRight = r2TrajectoryGeneratorRight.formatJointStateMsg(rrp2, 0)
        r2TrajectoryGeneratorLeft.moveToGoal(jointGoalLeft, 0.5, False)
        r2TrajectoryGeneratorRight.moveToGoal(jointGoalRight, 0.5, False)

    except rospy.ROSInterruptException:
        pass

