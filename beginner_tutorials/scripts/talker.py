#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler, quaternion_multiply

def readPose(fileName: str):
    poses = []
    meshPoses = []
    fileHanle = open(fileName, 'r')
    for line in fileHanle.readlines():
        linesplits = line.split(' ')
        if (len(linesplits) < 3):
            print("line read error ", line)
            continue
        pose = getPose(linesplits[0], linesplits[1], linesplits[2], linesplits[3], linesplits[4], linesplits[5], linesplits[6], linesplits[7])
        poses.append(pose)
        meshpose = getMeshPose(pose)
        meshPoses.append(meshpose)
    return poses, meshPoses

def getMeshPose(pose):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.MESH_RESOURCE
    marker.action = Marker.ADD
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = 1
    marker.color.r = 1
    marker.color.g = 1
    marker.color.b = 1
    marker.mesh_resource = "file:///home/junwangcas/code/controller_pisces/config/HMD.dae"
    marker.pose = pose.pose
    return marker

def getPose(timeStamp, x, y, z, qx, qy, qz, qw):
    pose = PoseStamped()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    pose.header.stamp.from_sec(float(timeStamp))

    # pose.pose.orientation = TransPose(pose.pose.orientation)
    pose.header.frame_id = 'map'

    return pose

def TransPose(raw_quaternion):
    trans_quaternion = [0, 1, 0, 0]
    # temp_q = quaternion_from_euler(0, 90.0 * 3.14 / 180.0, 0.0)
    trans_q = Quaternion()
    # trans_q.x = temp_q[0]
    # trans_q.y = temp_q[1]
    # trans_q.z = temp_q[2]
    # trans_q.w = temp_q[3]
    #
    # new_q = trans_q * raw_quaternion

    # temp_q = quaternion_from_euler(0, -90.0 * 3.14 / 180.0, 0.0)
    angle_90 = 90.0 * 3.14 / 180.0
    temp_q = quaternion_from_euler(-0, angle_90, 0)
    raw_q = [raw_quaternion.x, raw_quaternion.y, raw_quaternion.z, raw_quaternion.w]
    # new_q = temp_q * raw_q
    new_q = quaternion_multiply(temp_q, raw_q)
    new_q_msg = Quaternion(new_q[0], new_q[1], new_q[2], new_q[3])

    # new_q_msg = Quaternion(raw_q[0], raw_q[1], raw_q[2], raw_q[3])
    return new_q_msg


def pubPose(poses):
    pub = rospy.Publisher('/pubpose', PoseStamped, queue_size=10)
    rospy.init_node('pubposenode', anonymous=True)
    rate = rospy.Rate(10)
    i = 0
    while i < len(poses):
        posemsg = poses[i]
        pub.publish(posemsg)
        rate.sleep()
        print("pub")
        i = i + 1

def pubMeshPose(poses):
    pub = rospy.Publisher('/pubmeshpose', Marker, queue_size=10)
    rate = rospy.Rate(10)
    i = 0
    while i < len(poses):
        posemsg = poses[i]
        pub.publish(posemsg)
        rate.sleep()
        print("pub", i)
        i = i + 1

def pubAllPose(poses, mesh_poses):
    pubpose = rospy.Publisher('/pubpose', PoseStamped, queue_size=10)
    pub_meshpose = rospy.Publisher('/pubmeshpose', Marker, queue_size=10)
    rate = rospy.Rate(10)
    i = 0
    while i < len(poses):
        pubpose.publish(poses[i])
        # pub_meshpose.publish(mesh_poses[i])
        rate.sleep()
        print("pub", i)
        i = i + 1

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def pubMesh():
    pub = rospy.Publisher('meshmsg', Marker, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1
        marker.mesh_resource = "file:///home/junwangcas/code/controller_pisces/config/HMD.dae"
        pub.publish(marker)

if __name__ == '__main__':
    rospy.init_node('pubposenode', anonymous=True)
    filename = "/home/junwangcas/code/test_case_tools/launch_scripts/result_1659759634.57963/controller_2022-08-06-10-59-15/output.txt"
    poses, meshPoses = readPose(filename)
    pubAllPose(poses, meshPoses)
        # pubMesh()
