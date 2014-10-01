#!/usr/bin/env python
import roslib
import rospy
import tf
import numpy as np
import math
roslib.load_manifest('dviz_core')
import os
import csv
import rosbag
from dviz_core.msg import UserDemonstration
from dviz_core.msg import Step
from dviz_core.msg import Waypoint
from tf.transformations import euler_from_quaternion
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionFK
from kinematics_msgs.msg import PositionIKRequest
from sensor_msgs.msg import JointState
from arm_navigation_msgs.msg import RobotState
from arm_navigation_msgs.msg import MultiDOFJointState
from visualization_msgs.msg import Marker

OBJECT_NAMES = ["oil",
                "cake_mix",
                "water_bottle",
                "cake_pan",
                "hockey_stick",
                "eggs"]

ACTION_NAMES = ["pick",
                "place"]

DOF_NAMES = ["base_x",
             "base_y",
             "base_theta",
             "r_shoulder_pan_joint",
             "r_shoulder_lift_joint",
             "r_upper_arm_roll_joint",
             "r_elbow_flex_joint",
             "r_forearm_roll_joint",
             "r_wrist_flex_joint",
             "r_wrist_roll_joint"]

INITIAL_OBJECT_POSITIONS = [(-1.51785, 1.18239, 1.01662),    # oil
                            (-1.53422, 0.0206066, 1.47446),  # cake mix
                            (1.51495, 4.11129, 0.903346),    # water bottle
                            (0.480567, 1.58575, 0.702783),   # cake pan
                            (4.93094, 4.00069, 0.510713),    # hockey stick
                            (0.635449, 4.29259, 0.940121)]   # eggs

FINAL_OBJECT_POSITIONS = [(0.515593, 0.850125, 1.01283),     # oil
                          (0.520047, 1.17771, 1.00113),      # cake mix
                          (1.32377, 1.10262, 0.977923),      # water bottle
                          (0.901986, 0.799256, 0.939204),    # cake pan
                          (1.52564, 2.53245, 0.600653),      # hockey stick
                          (0.492781, 2.69086, 0.917871)]     # eggs

DEMOS = "/home/eratner/demonstrations/good"

FK_SERVICE = None
MARKER_PUBLISHER = None
VISUALIZATIONS = False
FRAME_RATE = 10.0

def normalizeAnglePositive(angle):
    return math.fmod(math.fmod(angle, 2.0 * math.pi) + 2.0 * math.pi, 2.0 * math.pi)

def normalizeAngle(angle):
    a = normalizeAnglePositive(angle)
    if a > math.pi:
        a -= 2.0 * math.pi

    return a

def shortestAngularDistance(fromAngle, toAngle):
    result = normalizeAnglePositive(normalizeAnglePositive(toAngle) - normalizeAnglePositive(fromAngle))

    if result > math.pi:
        result = -(2.0 * math.pi - result)

    return normalizeAngle(result)

def dist(posA, posB):
    return math.sqrt(pow(posA[0] - posB[0], 2) + pow(posA[1] - posB[1], 2) + pow(posA[2] - posB[2], 2))

def quatDist(qA, qB):
    #print str(2.0 * (qA[0] * qB[0] + qA[1] * qB[1] + qA[2] * qB[2] + qA[3] * qB[3]) - 1.0000001)
    x = 2.0 * (qA[0] * qB[0] + qA[1] * qB[1] + qA[2] * qB[2] + qA[3] * qB[3]) - 1.0
    if x > 1.0:
        x = 1.0
    elif x < -1.0:
        x = -1.0

    return math.acos(x)

def getPos(obj, action):
    if action == "Pick Up":
        return getInitialPos(obj)
    else:
        return getFinalPos(obj)

def getInitialPos(obj):
    index = OBJECT_NAMES.index(obj)
    return INITIAL_OBJECT_POSITIONS[index]

def getFinalPos(obj):
    index = OBJECT_NAMES.index(obj)
    return FINAL_OBJECT_POSITIONS[index]

def getBagfiles(directory):
    bagfiles = []
    for f in os.listdir(directory):
        if f.endswith(".bag"):
            bagfiles.append(os.path.join(directory, f))

    return bagfiles

# Returns the (stamped) pose of the given link in the base_link frame
def runFK(angles, linkName):
    #rospy.loginfo("Running FK for link " + linkName)
    header = rospy.Header()
    header.stamp = rospy.get_rostime()
    header.frame_id = "base_link"
    jointState = JointState(header, DOF_NAMES[3:], angles, [], [])
    try:
        response = FK_SERVICE(header, [linkName], RobotState(jointState, MultiDOFJointState()))
    except rospy.ServiceException, e:
        rospy.logerr("FK service call failed with error %s" % e)
        return None

    if response.error_code.val != 1:
        rospy.loginfo("FK failed!")
        return None

    return response.pose_stamped[0]

def toBaseFrame(position, basePose):
    baseTranslation = tf.transformations.translation_matrix([basePose.position.x,
                                                             basePose.position.y,
                                                             basePose.position.z])
    baseRotation = tf.transformations.quaternion_matrix([basePose.orientation.x,
                                                         basePose.orientation.y,
                                                         basePose.orientation.z,
                                                         basePose.orientation.w])
    T = tf.transformations.concatenate_matrices(baseTranslation, baseRotation)
    v = np.array([position[0],
                  position[1],
                  position[2],
                  1])
    prod = T.dot(np.transpose(v))
    return (prod[0], prod[1], prod[2])

def printWaypoint(waypoint):
    (r, p, y) = euler_from_quaternion([waypoint.base_pose.orientation.x,
                                       waypoint.base_pose.orientation.y,
                                       waypoint.base_pose.orientation.z,
                                       waypoint.base_pose.orientation.w])

    print 'base_x: {0}, base_y: {1}, base_yaw: {2},  shoulder_lift: {3}, upper_arm_roll: {4}, elbow_flex: {5}, forearm_roll: {6},  wrist_flex: {7}, wrist_roll: {8}'.format(
    waypoint.base_pose.position.x, waypoint.base_pose.position.y, y, waypoint.joint_states.position[0], waypoint.joint_states.position[1], waypoint.joint_states.position[2],
     waypoint.joint_states.position[3], waypoint.joint_states.position[4], waypoint.joint_states.position[5], waypoint.joint_states.position[6])

def isDifferent(waypointA, waypointB):
    diffX = abs(waypointA.base_pose.position.x - waypointB.base_pose.position.x)
    diffY = abs(waypointA.base_pose.position.y - waypointB.base_pose.position.y)
    (rollA, pitchA, yawA) = euler_from_quaternion([waypointA.base_pose.orientation.x,
                                                   waypointA.base_pose.orientation.y,
                                                   waypointA.base_pose.orientation.z,
                                                   waypointA.base_pose.orientation.w])
    (rollB, pitchB, yawB) = euler_from_quaternion([waypointB.base_pose.orientation.x,
                                                   waypointB.base_pose.orientation.y,
                                                   waypointB.base_pose.orientation.z,
                                                   waypointB.base_pose.orientation.w])
    yawDiff = abs(shortestAngularDistance(yawA, yawB))
    angleDiffs = []
    for i in range(7):
        angleDiffs.append(abs(shortestAngularDistance(waypointA.joint_states.position[i + 7], waypointB.joint_states.position[i + 7])))

    isDiff = False
    isDiff = isDiff or (diffX > 0.0)
    isDiff = isDiff or (diffY > 0.0)
    isDiff = isDiff or (yawDiff > 0.0)
    for i in range(7):
        isDiff = isDiff or (angleDiffs[i] > 0.0)

   # printWaypoint(waypointA)
    #printWaypoint(waypointB)
    #print 'diffx = ' + str(diffX) + ' diffy = ' + str(diffY) + ' diffyaw = ' + str(yawDiff) + ' jadiff = ' + str(angleDiffs)
    #print 'is diff? ' + str(isDiff)
    return isDiff

def parseBagfile(name):
    bag = rosbag.Bag(name, "r")

    demo = UserDemonstration()
    for topic, msg, t in bag.read_messages(topics = ["/demonstration"]):
        demo = msg

    #print "Parsing bagfile with " + str(len(demo.steps) - 1) + " goals completed" 
    data = []
    numSteps = 0
    for step in demo.steps:
        if VISUALIZATIONS:
            raw_input("Press any key to continue...")
        if step.object_type == "EOT" or step.object_type == "?" or step.step_duration.to_sec() <= 0:
            continue

        # stepData[0] = [object, action, total_time]
        # stepData[i][j] = total distance for DoF j, within region i, where i = (1, 2, 3, 4)
        #                  and j = (0, 1, 2, ..., 9) (one for each of the 10 DoFs)
        #                  note that region 4 is the DoF distances over the entire trajectory

        # stepData[i][j] = total distance for group j (where j = 0 for base xy group, j = 1 for base yaw group,
        #                  j = 2 for upper arm group, j = 3 for lower arm group)
        # stepData[0] = [object, action, total time, total time (no idle)]
        #stepData = [[0.0 for y in range(10)] for x in range(5)]
        #stepData = [[0.0 for y in range(4)] for x in range(11)]
        stepData = [[0.0 for y in range(4)] for x in range(5)]
        stepData[0] = [step.object_type, step.action, step.step_duration.to_sec(), 0.0]
        numWaypoints = len(step.waypoints)
        if numWaypoints > 0:
            # Get the initial position of the base
            initialBasePos = (step.waypoints[0].base_pose.position.x,
                              step.waypoints[0].base_pose.position.y,
                              step.waypoints[0].base_pose.position.z)
            (baseRoll, basePitch, baseYaw) = euler_from_quaternion([step.waypoints[0].base_pose.orientation.x,
                                                                    step.waypoints[0].base_pose.orientation.y,
                                                                    step.waypoints[0].base_pose.orientation.z,
                                                                    step.waypoints[0].base_pose.orientation.w])
            if VISUALIZATIONS:
                baseMarker = Marker()
                baseMarker.header.frame_id = "map"
                baseMarker.header.stamp = rospy.Time.now()
                baseMarker.type = Marker.CUBE
                baseMarker.ns = name + "_initial_markers"
                baseMarker.id = numSteps * 3
                baseMarker.pose.position.x = initialBasePos[0]
                baseMarker.pose.position.y = initialBasePos[1]
                baseMarker.pose.position.z = initialBasePos[2]
                baseMarker.pose.orientation.x = step.waypoints[0].base_pose.orientation.x
                baseMarker.pose.orientation.y = step.waypoints[0].base_pose.orientation.y
                baseMarker.pose.orientation.z = step.waypoints[0].base_pose.orientation.z
                baseMarker.pose.orientation.w = step.waypoints[0].base_pose.orientation.w
                baseMarker.scale.x = 0.55
                baseMarker.scale.y = 0.55
                baseMarker.scale.z = 0.55
                baseMarker.color.a = 1.0
                baseMarker.color.r = 0.0
                baseMarker.color.g = 1.0
                baseMarker.color.b = 0.0
                MARKER_PUBLISHER.publish(baseMarker)

            #Get the initial position of the right gripper
            gripperPoseStamped = runFK(step.waypoints[0].joint_states.position[7:14],
                                       "r_wrist_roll_link")
            if gripperPoseStamped is None:
                print "FK failed; must skip this step"
                continue

            # The pose of the right gripper is returned in the base_link frame, so must be 
            # transformed back into the map frame
            initialGripperPos = toBaseFrame([gripperPoseStamped.pose.position.x,
                                             gripperPoseStamped.pose.position.y,
                                             gripperPoseStamped.pose.position.z],
                                            step.waypoints[0].base_pose)

            if VISUALIZATIONS:
                gripperMarker = Marker()
                gripperMarker.header.frame_id = "map"
                gripperMarker.header.stamp = rospy.Time.now()
                gripperMarker.type = Marker.SPHERE
                gripperMarker.ns = name + "_initial_markers"
                gripperMarker.id = numSteps * 3 + 1
                gripperMarker.pose.position.x = initialGripperPos[0]
                gripperMarker.pose.position.y = initialGripperPos[1]
                gripperMarker.pose.position.z = initialGripperPos[2]
                gripperMarker.pose.orientation.x = 0
                gripperMarker.pose.orientation.y = 0
                gripperMarker.pose.orientation.z = 0
                gripperMarker.pose.orientation.w = 1
                gripperMarker.scale.x = 0.35
                gripperMarker.scale.y = 0.35
                gripperMarker.scale.z = 0.35
                gripperMarker.color.a = 1.0
                gripperMarker.color.r = 0.0
                gripperMarker.color.g = 1.0
                gripperMarker.color.b = 0.0
                MARKER_PUBLISHER.publish(gripperMarker)

            # Get the position of the goal
            goalPos = (0, 0, 0)
            if step.action == 'Pick Up':
                print 'PICK UP'
                goalPos = (step.grasp.position.x,
                           step.grasp.position.y,
                           step.grasp.position.z)
            else:
                print 'PLACE'
                goalPos = getFinalPos(step.object_type)

            if VISUALIZATIONS:
                gripperMarker = Marker()
                gripperMarker.header.frame_id = "map"
                gripperMarker.header.stamp = rospy.Time.now()
                gripperMarker.type = Marker.SPHERE
                gripperMarker.ns = name + "_initial_markers"
                gripperMarker.id = numSteps * 3 + 2
                gripperMarker.pose.position.x = goalPos[0]
                gripperMarker.pose.position.y = goalPos[1]
                gripperMarker.pose.position.z = goalPos[2]
                gripperMarker.pose.orientation.x = 0
                gripperMarker.pose.orientation.y = 0
                gripperMarker.pose.orientation.z = 0
                gripperMarker.pose.orientation.w = 1
                gripperMarker.scale.x = 0.35
                gripperMarker.scale.y = 0.35
                gripperMarker.scale.z = 0.35
                gripperMarker.color.a = 1.0
                gripperMarker.color.r = 1.0
                gripperMarker.color.g = 0.0
                gripperMarker.color.b = 0.0
                MARKER_PUBLISHER.publish(gripperMarker)

            # "Clean" the list of waypoints so that it includes no idle waypoints
            # wpts = [step.waypoints[0]]
            # numWpts = 1
            # k = 1
            # while k < len(step.waypoints):
            #     if isDifferent(wpts[numWpts - 1], step.waypoints[k]):
            #         wpts.append(step.waypoints[k])
            #         numWpts += 1

            #     k += 1
            numWpts = numWaypoints
            wpts = step.waypoints

            print 'Num waypoints changed from ' + str(numWaypoints) + ' to ' + str(numWpts)
            # For each waypoint, determine which region the current position falls in:
            # ( INITIAL ---- I ----- | -------- II --------- | --- III --- GOAL )
            # I is the start region (stepData[1])
            # II is the region between start and goal ranges (stepData[2])
            # III is the goal region (stepData[3])
            for i in range(numWpts - 1):
                #percentile = int((float(i) / numWpts) * 100.0) / 10
                #print 'Percentile = ' + str(percentile)
                basePos = (wpts[i].base_pose.position.x,
                           wpts[i].base_pose.position.y,
                           wpts[i].base_pose.position.z)
                (baseRoll, basePitch, baseYaw) = euler_from_quaternion([wpts[i].base_pose.orientation.x,
                                                                        wpts[i].base_pose.orientation.y,
                                                                        wpts[i].base_pose.orientation.z,
                                                                        wpts[i].base_pose.orientation.w])

                # Get the pose of the gripper at waypoints i and i + 1
                firstGripperPoseStamped = runFK(step.waypoints[i].joint_states.position[7:14],
                                           "r_wrist_roll_link")
                if firstGripperPoseStamped is None:
                    print "FK failed; must skip this step"
                    continue

                # The pose of the right gripper is returned in the base_link frame, so must be 
                # transformed back into the map frame
                firstGripperPos = toBaseFrame([firstGripperPoseStamped.pose.position.x,
                                               firstGripperPoseStamped.pose.position.y,
                                               firstGripperPoseStamped.pose.position.z],
                                              step.waypoints[i].base_pose)



                regions = []
                # Get the distance from the initial position to the current position
                distInitialToCurrent = dist(initialGripperPos, firstGripperPos)
                # Get the distance from the current position to the goal position
                distCurrentToGoal = dist(firstGripperPos, goalPos)
                #if distCurrentToGoal < 0.5:
                #    print 'Close to goal: ' + str(distCurrentToGoal)

                if distInitialToCurrent <= 1.0 or distCurrentToGoal <= 1.0: # In region I or III (or possibly both)
                    if distInitialToCurrent <= 1.0:
                        regions.append(1)
                    if distCurrentToGoal <= 1.0:
                        regions.append(3)
                    if len(regions) == 2:
                        print 'In both start and goal regions'
                else:
                    regions.append(2)

                regions.append(4)

                if VISUALIZATIONS:
                    gripperMarker = Marker()
                    gripperMarker.header.frame_id = 'map'
                    gripperMarker.header.stamp = rospy.Time.now()
                    gripperMarker.type = Marker.SPHERE
                    gripperMarker.ns = name + '_step_' + str(numSteps) + '_gripper_waypoint'
                    gripperMarker.id = i
                    gripperMarker.pose.position.x = firstGripperPos[0]
                    gripperMarker.pose.position.y = firstGripperPos[1]
                    gripperMarker.pose.position.z = firstGripperPos[2]
                    gripperMarker.pose.orientation.x = 0
                    gripperMarker.pose.orientation.y = 0
                    gripperMarker.pose.orientation.z = 0
                    gripperMarker.pose.orientation.w = 1
                    gripperMarker.scale.x = 0.2
                    gripperMarker.scale.y = 0.2
                    gripperMarker.scale.z = 0.2
                    gripperMarker.color.a = 1.0
                    gripperMarker.color.r = 1.0
                    gripperMarker.color.g = 1.0
                    gripperMarker.color.b = 0.0
                    MARKER_PUBLISHER.publish(gripperMarker)
                    baseMarker = Marker()
                    baseMarker.header.frame_id = 'map'
                    baseMarker.header.stamp = rospy.Time.now()
                    baseMarker.type = Marker.CUBE
                    baseMarker.ns = name + '_step_' + str(numSteps) + '_base_waypoint'
                    baseMarker.id = i
                    baseMarker.pose.position.x = basePos[0]
                    baseMarker.pose.position.y = basePos[1]
                    baseMarker.pose.position.z = basePos[2]
                    baseMarker.pose.orientation.x = wpts[i].base_pose.orientation.x
                    baseMarker.pose.orientation.y = wpts[i].base_pose.orientation.y
                    baseMarker.pose.orientation.z = wpts[i].base_pose.orientation.z
                    baseMarker.pose.orientation.w = wpts[i].base_pose.orientation.w
                    baseMarker.scale.x = 0.45
                    baseMarker.scale.y = 0.45
                    baseMarker.scale.z = 0.45
                    baseMarker.color.a = 1.0
                    baseMarker.color.r = 1.0
                    baseMarker.color.g = 1.0
                    baseMarker.color.b = 0.0
                    MARKER_PUBLISHER.publish(baseMarker)

                secondGripperPoseStamped = runFK(step.waypoints[i + 1].joint_states.position[7:14],
                                           "r_wrist_roll_link")
                if secondGripperPoseStamped is None:
                    print "FK failed; must skip this step"
                    continue

                # The pose of the right gripper is returned in the base_link frame, so must be 
                # transformed back into the map frame
                secondGripperPos = toBaseFrame([secondGripperPoseStamped.pose.position.x,
                                                secondGripperPoseStamped.pose.position.y,
                                                secondGripperPoseStamped.pose.position.z],
                                               step.waypoints[i + 1].base_pose)

                for region in regions:
                    # Group 0: Base xy
                    initialBase = (wpts[i].base_pose.position.x, wpts[i].base_pose.position.y, 0.0)
                    finalBase = (wpts[i + 1].base_pose.position.x, wpts[i + 1].base_pose.position.y, 0.0)
                    stepData[region][0] += dist(initialBase, finalBase)

                    # Group 1: Base yaw
                    (initialRoll, initialPitch, initialYaw) = euler_from_quaternion([wpts[i].base_pose.orientation.x,
                                                                                     wpts[i].base_pose.orientation.y,
                                                                                     wpts[i].base_pose.orientation.z,
                                                                                     wpts[i].base_pose.orientation.w])
                    (finalRoll, finalPitch, finalYaw) = euler_from_quaternion([wpts[i + 1].base_pose.orientation.x,
                                                                               wpts[i + 1].base_pose.orientation.y,
                                                                               wpts[i + 1].base_pose.orientation.z,
                                                                               wpts[i + 1].base_pose.orientation.w])
                    stepData[region][1] += abs(shortestAngularDistance(initialYaw, finalYaw))


                    # Group 2: End-effector xyz
                    first = (firstGripperPoseStamped.pose.position.x,
                             firstGripperPoseStamped.pose.position.y,
                             firstGripperPoseStamped.pose.position.z)
                    second = (secondGripperPoseStamped.pose.position.x,
                              secondGripperPoseStamped.pose.position.y,
                              secondGripperPoseStamped.pose.position.z)
                             
                    #stepData[region][2] += dist(firstGripperPos, secondGripperPos)
                    stepData[region][2] += dist(first, second)

                    # Group 3: End-effector orientation
                    stepData[region][3] += quatDist([firstGripperPoseStamped.pose.orientation.x, 
                                                     firstGripperPoseStamped.pose.orientation.y,
                                                     firstGripperPoseStamped.pose.orientation.z,
                                                     firstGripperPoseStamped.pose.orientation.w],
                                                    [secondGripperPoseStamped.pose.orientation.x,
                                                     secondGripperPoseStamped.pose.orientation.y,
                                                     secondGripperPoseStamped.pose.orientation.z,
                                                     secondGripperPoseStamped.pose.orientation.w])
                    # (eefInitialRoll, eefInitialPitch, eefInitialYaw) = euler_from_quaternion([firstGripperPoseStamped.pose.orientation.x,
                    #                                                                           firstGripperPoseStamped.pose.orientation.y,
                    #                                                                           firstGripperPoseStamped.pose.orientation.z,
                    #                                                                           firstGripperPoseStamped.pose.orientation.w])
                    # (eefFinalRoll, eefFinalPitch, eefFinalYaw) = euler_from_quaternion([secondGripperPoseStamped.pose.orientation.x,
                    #                                                                     secondGripperPoseStamped.pose.orientation.y,
                    #                                                                     secondGripperPoseStamped.pose.orientation.z,
                    #                                                                     secondGripperPoseStamped.pose.orientation.w])
                    # stepData[region][3] += abs(shortestAngularDistance(eefInitialRoll, eefFinalRoll))
                    # stepData[region][3] += abs(shortestAngularDistance(eefInitialPitch, eefFinalPitch))
                    # stepData[region][3] += abs(shortestAngularDistance(eefInitialYaw, eefFinalYaw))

        stepData[0][3] = float(numWpts) / FRAME_RATE
        data.append(stepData)
        numSteps += 1

    print "Processed " + str(numSteps) + " goals completed"
    return numSteps, data

def main():
    directories = [DEMOS]
    numBagfiles = 0
    numGoalsCompleted = 0
    data = []
    for directory in directories:
        print "Looking in directory " + directory
        bagfiles = getBagfiles(directory)
        print "Found " + str(len(bagfiles)) + " bagfiles in this directory"
        numBagfiles += len(bagfiles)
        for bagfile in bagfiles:
            n, bagData = parseBagfile(bagfile)
            for entry in bagData:
                data.append(entry)
            numGoalsCompleted += n

    print "====="
    print "Completed a total of " + str(numGoalsCompleted) + " goals"
    print "====="

    # Write the data to a CSV file
    print "Writing " + str(len(data)) + " entries to CSV file"
    with open("output_joint_data.csv", "w") as f:
        writer = csv.writer(f, delimiter=",")
        #writer.writerow(["object", "action", "duration (s)", "actual duration (s)"] + ["base xy group", "base yaw group", "upper arm group", "lower arm group"] * 10)
        writer.writerow(["object", "action", "duration (s)", "actual duration (s)"] + ["base xy", "base yaw", "eef position", "eef orientation"] * 4)
        rows = [x[0] + x[1] + x[2] + x[3] + x[4] for x in data]
        writer.writerows(rows)

    # For the PCA analysis
    #directories = [DEMOS]
    #allBagfiles = []
    #for directory in directories:
    #print "Looking in directory " + directory
    #bagfiles = getBagfiles(directory)
    #print "Found " + str(len(bagfiles)) + " bagfiles in this directory"
    #bagfiles = bagfiles[:40]
    #allBagfiles += bagfiles
    
    #pcaParseBagfiles(allBagfiles)

if __name__ == "__main__":
    rospy.init_node("dviz_stats")

    # Initialize FK service to get pose of eef
    FK_SERVICE = rospy.ServiceProxy("/pr2_right_arm_kinematics/get_fk", GetPositionFK, True)
    rospy.loginfo("Waiting for PR2 right arm FK service...")
    rospy.wait_for_service("/pr2_right_arm_kinematics/get_fk")
    rospy.loginfo("...FK service found.")

    # Initialize a marker publisher for debugging
    MARKER_PUBLISHER = rospy.Publisher("/visualization_marker", Marker)

    main()
