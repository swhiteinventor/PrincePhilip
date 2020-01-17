#! /usr/bin/env python
'''
Adapted from Relaxed_IK work
author: Danny Rakita
website: http://pages.cs.wisc.edu/~rakita/
email: rakita@cs.wisc.edu
last update: 7/1/18

Designed to subscribe to a topic containing end-effector pose of one hand, then calculating the pose of the other hand to achieve camera following.
'''
######################################################################################################
import os
import sys
import rospy

from relaxed_ik.msg import EEPoseGoals, JointAngles
from std_msgs.msg import Float32
# from relaxed_ik.Utils.colors import bcolors
ebolabot_root = os.getenv("EBOLABOT_PATH",".")
sys.path.append(ebolabot_root)
from Common.system_config import EbolabotSystemConfig
# import relaxed_ik
sys.path.append('/home/trina/PrincePhilip/src/RelaxedIK/')
from relaxedIK import RelaxedIK
from baxter_core_msgs.msg import EndpointState


#eepg = None
#def eePoseGoals_cb(data):
#    global eepg
#    eepg = data
def callback_pose(data):
     global hand
     hand = data # [data.pose.position.x, data.pose.position.y, data.pose.position.z]

if __name__ == '__main__':
    rospy.init_node('relaxed_ik_node')
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions',JointAngles ,queue_size=3)
    rospy.Subscriber('/robot/limb/left/endpoint_state', EndpointState, callback_pose)
    # TODO where does EndpointState get imported from?
    rospy.sleep(0.3)

    config_file_name = rospy.get_param('config_file_name', default='relaxedIK.config')
    my_relaxedIK = RelaxedIK.init_from_config(config_file_name)
    num_chains = my_relaxedIK.vars.robot.numChains

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        # pose_goals = eepg.ee_poses
        header = data.header
        p = data.pose
        # num_poses = len(pose_goals)
        # if not num_poses == num_chains:
            # print bcolors.FAIL + 'ERROR: Number of pose goals ({}) ' \                                 'not equal to the number of kinematic chains ({}).  Exiting relaxed_ik_node'.format(num_poses, num_chains)
            # rospy.signal_shutdown()

        pos_goals = []
        quat_goals = []

        #for p in pose_goals:
        
        pos_x = p.position.x
        pos_y = p.position.y
        pos_z = p.position.z

        quat_w = p.orientation.w
        quat_x = p.orientation.x
        quat_y = p.orientation.y
        quat_z = p.orientation.z

        pos_goals.append([pos_x, pos_y, pos_z])
        quat_goals.append([quat_w, quat_x, quat_y, quat_z])

        xopt = my_relaxedIK.solve(pos_goals, quat_goals)
        ja = JointAngles()
        ja.header = header
        for x in xopt:
            ja.angles.append(Float32(x))

        angles_pub.publish(ja)

        rate.sleep()





