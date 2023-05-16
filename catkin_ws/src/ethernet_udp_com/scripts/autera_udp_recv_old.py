#!/usr/bin/env python

import numpy as np
import rospy
import socket
import struct
import sys
import signal
import tf
import tf.msg
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

class topic_talker:
    """class for publishing information received via ethernet udp
    """
    def __init__(self):
        """
        register interrupt handler for CTRL+C
        initialize socket
        initialize node
        initialize publisher
        
        udp config
        sending always from port 50xxx
        receiving always on port 51xxx
        """
        signal.signal(signal.SIGINT, self.signal_handler)
        # udp config -> receiver needs to be specified
        # receiver config lenovo e560 thab
        # self.UDP_IP = "192.168.1.2"
        # self.UDP_PORT = 51100
        # receiver config autera agv387 thab
        self.UDP_IP = "192.168.1.3"
        self.UDP_PORT = 51100
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        # rate of udp receiver node
        self.udp_rate = 50
        # queue if subscriber is not fast enough
        # TODO: change queue size to 1 -> no queue so only current values are used
        self.pub_mab_pose = rospy.Publisher("mab_pose", PoseStamped, queue_size=10)
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=10)
        self.pub_mab_trajectory = rospy.Publisher("mab_trajectory", MarkerArray, queue_size=10)
        self.pub_mab_trajectory_warn = rospy.Publisher("mab_trajectory_warn", MarkerArray, queue_size=10)
        self.pub_mab_trajectory_stop = rospy.Publisher("mab_trajectory_stop", MarkerArray, queue_size=10)
        # init node
        # anonymous adds random numbers for unique name
        rospy.init_node('udp_receiver', anonymous=True)
        rospy.loginfo("udp_receiver: initialized")
        self.udp_listen()

    def udp_listen(self):
        """
        receive udp messages, unpack and publish to topic
        """
        rospy.loginfo("udp_receiver: listening for udp messages...")
        rate = rospy.Rate(self.udp_rate)
        while not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(2048)
                if data != []:
                    data_len = len(data)
                    # mab pose msg with transmission check
                    if data_len == 20 and data[0:4] == data[16:20]:
                        data = struct.unpack('ifffi', data)
                        self.publish_mab_pose(data[1:4])
                    # trajectory with transmission check -> 24 trajectory points transmitted -> 4 values each -> 380 bytes + 2*tc + dir + rest
                    elif data_len == 496 and data[0:4] == data[492:496]:
                        dataFormat = f"{120}f" # 30*4 = 120
                        data_dir = struct.unpack('f', data[4:8])
                        data_rest = struct.unpack('f', data[8:12])
                        data_trajectory = struct.unpack(dataFormat, data[12:492])
                        self.publish_mab_trajectory(data_dir, data_rest, data_trajectory)
                    else:
                        rospy.loginfo("udp_receiver: problem with msg length %s", data_len)
                rate.sleep()
            except rospy.ROSInterruptException:
                self.sock.close()

    def publish_mab_pose(self, pose_data):
        """function for publishing the vehicles pose information
        published as message type Pose
        Args:
            v_pose ([float float float]): array with x [m], y [m], psi [rad]
        """
        # check if program is running
        if not rospy.is_shutdown():
            # data
            x = pose_data[0]
            y = pose_data[1]
            # numpy cos: element wise in rad
            ori_z = np.sin(pose_data[2]/2)
            ori_w = np.cos(pose_data[2]/2)
            # pose
            p = PoseStamped()
            p.header.frame_id = "global"
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = np.sin(pose_data[2]/2)
            p.pose.orientation.w = np.cos(pose_data[2]/2)
            # dynamic transform msg
            t = TransformStamped()
            t.header.frame_id = "global"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "forklift"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = np.sin((pose_data[2])/2)
            t.transform.rotation.w = np.cos((pose_data[2])/2)

            tfmsg = tf.msg.tfMessage([t])
            # 3 functions
            # print to screen
            # print to log file
            # print to rosout (with rqt_console)
            # rospy.loginfo("udp_msg_receiver: udp msg published")
            # publish message
            self.pub_mab_pose.publish(p)
            self.pub_tf.publish(tfmsg)
            
    def publish_mab_trajectory(self, data_dir, data_rest, data_path):
        """function for publishing the vehicles velocity setpoint
        published as message type Float32
        Args:
            data (_type_): _description_
        """
        nSetpoints = 30
        dir = data_dir[0]
        rest = data_rest[0]
        markerArray = MarkerArray()
        markerArray_warn = MarkerArray()
        markerArray_stop = MarkerArray()
        i = 0
        # check if program is running and if range of chi (data_path[i*4+3]) is valid
        while not rospy.is_shutdown() and i < nSetpoints and data_path[i*4+3] < 10:
            # get values
            x_set = data_path[i*4]
            y_set = data_path[i*4+1]
            yaw_set = data_path[i*4+2]
            chi = data_path[i*4+3]
            ori_z = np.sin(yaw_set/2)
            ori_w = np.cos(yaw_set/2)
            # marker init
            marker = Marker()
            marker_warn = Marker()
            marker_stop = Marker()
            # fill marker with values
            marker.id = i
            marker.ns = 'trajectory'
            marker.type = 2
            marker.header.frame_id = "global"
            marker.pose.position.x = x_set
            marker.pose.position.y = y_set
            marker.pose.position.z = 0.0
            marker.pose.orientation.z = ori_z
            marker.pose.orientation.w = ori_w
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = chi
            marker.color.g = float(dir)
            marker.color.b = float(rest)
            marker.lifetime = rospy.Duration(secs=1)
            markerArray.markers.append(marker)
            # marker for warn distance
            marker_warn.id = i
            marker_warn.ns = 'trajectory_warn'
            marker_warn.type = 2
            marker_warn.header.frame_id = "global"
            marker_warn.pose.position.x = x_set
            marker_warn.pose.position.y = y_set
            marker_warn.pose.orientation.z = ori_z
            marker_warn.pose.orientation.w = ori_w
            marker_warn.scale.x = 2 * (5 * abs(chi) + 2)
            marker_warn.scale.y = 2 * (5 * abs(chi) + 2)
            marker_warn.scale.z = 0.025
            marker_warn.color.a = 0.2
            marker_warn.color.r = 0.5
            marker_warn.color.g = 0.5
            marker_warn.color.b = 0.0
            marker_warn.lifetime = rospy.Duration(secs=1)
            markerArray_warn.markers.append(marker_warn)
            # marker for stop distance
            marker_stop.id = i
            marker_stop.ns = 'trajectory_stop'
            marker_stop.type = 2
            marker_stop.header.frame_id = "global"
            marker_stop.pose.position.x = x_set
            marker_stop.pose.position.y = y_set
            marker_stop.pose.position.z = 0.0
            marker_stop.pose.orientation.z = ori_z
            marker_stop.pose.orientation.w = ori_w
            marker_stop.scale.x = 2 * (5 * abs(chi) + 1)
            marker_stop.scale.y = 2 * (5 * abs(chi) + 1)
            marker_stop.scale.z = 0.05
            marker_stop.color.a = 1.0
            marker_stop.color.r = 1.0
            marker_stop.color.g = 0.0
            marker_stop.color.b = 0.0
            marker_stop.lifetime = rospy.Duration(secs=1)
            markerArray_stop.markers.append(marker_stop)
            i += 1
        self.pub_mab_trajectory.publish(markerArray)
        self.pub_mab_trajectory_warn.publish(markerArray_warn)
        self.pub_mab_trajectory_stop.publish(markerArray_stop)


    def signal_handler(self, sig, frame):
        """handler for CTRL+C
        socket.recvfrom is blocking programm -> use signal_handler with additional process for closing socket
        Args:
            sig (_type_): _description_
            frame (_type_): _description_
        """
        # rospy.loginfo("udp_msg_receiver: signal_handler terminated process")
        self.sock.close()

if __name__ == '__main__':
    # create talker
    udp_msg_receiver = topic_talker()
