#!/usr/bin/env python

import random
import socket
import struct
import numpy as np
import rospy
import tf
from ethernet_udp_com.msg import trajectory
from geometry_msgs.msg import PoseStamped
# from reflector_finder.msg import reflectors
from std_msgs.msg import String, Float32
from visualization_msgs.msg import Marker, MarkerArray


class topic_listener:
    """class for subscribing to topic and forwarding the messages via ethernet udp
    """
    def __init__(self):
        """configuration
        socket:
        sending port: 50xxx
        receiving port: 51xxx
        ros:
        initialize node
        initialize subscriber
        """
        self.counter_ref_meas = 0
        self.counter_v_set = 0
        self.counter_pose = 0
        # udp config -> sender needs to be specified in mab simulink software
        # sender config (lenovo e560 thab)
        # self.UDP_IP = "192.168.1.2"
        # self.UDP_PORT = 50100
        # sender config (autera agv387 thab)
        self.UDP_IP = "192.168.1.3"
        self.UDP_PORT = 50100
        # receiver config (mab on agv387)
        self.UDP_IP_TARGET = "192.168.1.200"
        self.UDP_PORT_TARGET = 51200
        # socket configuration
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.UDP_IP, self.UDP_PORT))
        # initialize subscriber node
        rospy.init_node('udp_sender', anonymous=True)
        rospy.loginfo('udp_sender: initialized')
        # define subscription
        # rospy.Subscriber("autera_reflector_list", reflectors, self.callback_reflector_list)
        # rospy.Subscriber("autera_trajectory", trajectory, self.callback_trajectory)
        rospy.Subscriber("autera_v_set", Float32, self.callback_v_set)
        rospy.Subscriber("pose/odom2base", PoseStamped, self.callback_pose)
        # callback deactivated for alternative structure
        # checking for obstracles on mab -> use this subscriber -> send obj data to mab
        # rospy.Subscriber("/perception/objects", MarkerArray, self.callback_obj)
        rospy.loginfo('udp_sender: subscriptions registered')
        
    def incrementCounter(self, counter):
        '''
        function returns incremented counter in range 0...99
        used for transmission check v2 -> incrementing numbers as transmission check
        missing data can be recognized
        '''
        max_counter = 999
        counter = counter + 1
        if counter > max_counter:
            counter = 0
        return counter    
        
    def callback_v_set(self, data):
        # packing from message datatype to bytes for sending
        # message length
        # 1*4 + 1*4 + 1*4 = 12 bytes
        try:
            self.counter_v_set = self.incrementCounter(self.counter_v_set)
            data_send_tc = struct.pack('i', self.counter_v_set)
            # alternative sending mode with same message lenght as reflector message
            n_ref = self.mapStorage.n_ref_max
            dummy_array = [0.0]*n_ref
            dummy_array[0] = data.data
            data_send_tc = struct.pack('i', self.counter_v_set)
            data_send_status = struct.pack('iii', -1, 0, 0)
            dataVector = f"{n_ref*2}f"
            data_send_list_dummy = struct.pack(dataVector, *dummy_array, *dummy_array)
            data_send = data_send_tc + data_send_status  + data_send_list_dummy  + data_send_list_dummy + data_send_tc
            # sending data via udp to mab
            self.sock.sendto(data_send, (self.UDP_IP_TARGET, self.UDP_PORT_TARGET))
            # rospy.loginfo('udp_msg_sender: v_set msg sent')
        except:
            rospy.loginfo('udp_msg_sender: problem with sending data')
        
    def callback_reflector_list(self, data):
        """callback function for topic 'autera_reflector_list'
        forwards via udp if message is published on this topic
        message is packed with struct.pack

        Args:
            data (reflectors): message from topic
        """
        # TODO: documentation for udp message
        # get data from topic 'autera_reflector_list'
        reflector_status = data.reflector_status
        n_reflectors = data.n_reflectors
        n_reflectors_visible = data.n_reflectors_visible
        # initialize reflector arrays -> 0.0 when not detected
        i = 0
        # initialize reflector arrays -> 0.0 when not detected
        x_ref_meas = [0.0]*n_reflectors
        y_ref_meas = [0.0]*n_reflectors
        x_ref_map = [0.0]*n_reflectors
        y_ref_map = [0.0]*n_reflectors
        # rospy.loginfo('n_ref: %i', n_reflectors)
        while i < n_reflectors:
            # rospy.loginfo('i: %i', i)
            # rospy.loginfo('length x_ref: %i', len(x_reflector))
            if i < len(data.x_ref_meas):
                x_ref_meas[i] = data.x_ref_meas[i]
                y_ref_meas[i] = data.y_ref_meas[i]

            if i < len(data.x_ref_map):
                x_ref_map[i] = data.x_ref_map[i]
                y_ref_map[i] = data.y_ref_map[i]
            else:
                break
            i = i + 1
        # packing from message datatype to bytes for sending
        # message length -> n_reflectors = 10 -> 
        # 1*4 + 3*4 + n_ref*2*2*4 + 1*4 = 180 bytes
        try:
            dataVector = f"{n_reflectors*2}f"
            # transmission check works with 0, but not with other numbers
            transmission_check = random.randint(0, 9)
            data_send_tc= struct.pack('i', transmission_check)
            data_send_status = struct.pack('iii', reflector_status, n_reflectors, n_reflectors_visible)
            data_send_list_meas = struct.pack(dataVector, *x_ref_meas, *y_ref_meas)
            data_send_list_map = struct.pack(dataVector, *x_ref_map, *y_ref_map)
            data_send = data_send_tc + data_send_status  + data_send_list_meas  + data_send_list_map + data_send_tc
        except:
            rospy.loginfo('udp_msg_sender: problem with received data - list length')
        # sending data via udp to mab
        self.sock.sendto(data_send, (self.UDP_IP_TARGET, self.UDP_PORT_TARGET))
        # rospy.loginfo("udp_msg_sender: sending reflector msg")

    def callback_trajectory(self, data):
        """"callback function for topic 'autera_trajectory'
        forwards via udp if message is published on this topic
        message is packed with struct.pack

        Args:
            data (trajectory): message from topic
        """
        # TODO: documentation for udp message
        # get data from topic 'autera_trajectory'
        message_id = data.message_id
        number_setpoint = data.number_setpoint
        max_number_setpoints = data.max_number_setpoints
        x_set = data.x_set
        y_set = data.y_set
        psi_set = data.psi_set
        s_k_set = data.s_k_set
        chi_set = data.chi_set
        v_set = data.v_set
        dir_set = data.dir_set
        mast_set = data.mast_set
        # packing from message datatype to bytes for sending
        # message length = 11*4bytes = 44bytes
        data_send = struct.pack('iiiffffffif', message_id, number_setpoint,
        max_number_setpoints, x_set, y_set, psi_set, s_k_set, chi_set, v_set,
        dir_set, mast_set)
        # sending data via udp to mab
        self.sock.sendto(data_send, (self.UDP_IP_TARGET, self.UDP_PORT_TARGET))
        # rospy.loginfo("udp_msg_sender: sending trajectory msg")

    def callback_obj(self, data):
        """"callback function for topic 'x'
        forwards via udp if message is published on this topic
        message is packed with struct.pack

        Args:
            data (x): message from topic
        """
        # message type is MarkerArray
        # max number of objects -> 40
        # msg: data:10*8*4+transmission-check:4*2+data-information:4*4 = 344
        n_obj_max = 40
        n_obj_visible = 0
        obj_array = np.zeros(shape=(800,1)) # []
        # read data from msg
        n_obj_visible = len(data.markers)
        if n_obj_visible > n_obj_max:
            n_obj_visible = n_obj_max
        # fill in visible objects in array
        for n_obj in range(n_obj_visible):
            if n_obj > n_obj_max-1:
                break
            current_marker = data.markers[n_obj]
            current_x = current_marker.pose.position.x
            current_y = current_marker.pose.position.y
            current_dx = current_marker.scale.x * 0.5
            current_dy = current_marker.scale.y * 0.5
            obj_array[n_obj*8] = current_x + current_dx # top right x coordinate
            obj_array[n_obj*8+1] = current_y + current_dy # top right x coordinate
            obj_array[n_obj*8+2] = current_x - current_dx # top left x coordinate
            obj_array[n_obj*8+3] = current_y + current_dy # top left x coordinate
            obj_array[n_obj*8+4] = current_x - current_dx # bottom left x coordinate
            obj_array[n_obj*8+5] = current_y - current_dy # bottom left x coordinate
            obj_array[n_obj*8+6] = current_x + current_dx # bottom right x coordinate
            obj_array[n_obj*8+7] = current_y - current_dy # bottom right x coordinate
        # packing from message datatype to bytes for sending
        try:
            for i in range(4):
                obj_array_send = obj_array[i*80:80+i*80]
                dataVector = f"{n_obj_max*2}f"
                transmission_check = random.randint(0, 99)
                # data_count must be increased by one to work with matlab
                data_count = struct.pack('i', i+1)
                data_max = struct.pack('i', 4)
                data_n_obj_visible = struct.pack('i', n_obj_visible)
                data_send_tc= struct.pack('i', transmission_check)
                data_send_status = struct.pack('i', n_obj_visible)
                data_send_list_obj = struct.pack(dataVector, *obj_array_send)
                data_send = data_send_tc + data_count + data_max + data_n_obj_visible + data_send_status + data_send_list_obj + data_send_tc
                # sending data via udp to mab
                self.sock.sendto(data_send, (self.UDP_IP_TARGET, self.UDP_PORT_TARGET))
                # rospy.loginfo("udp_msg_sender: sending object msg")
        except:
            rospy.loginfo('udp_msg_sender: problem with sending data - list length / msg packing / udp connection')

    def callback_pose(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        quaternion = (
            data.pose.orientation.x,
            data.pose.orientation.y,
            data.pose.orientation.z,
            data.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]
        try:
            self.counter_v_set = self.incrementCounter(self.counter_pose)
            data_send_tc = struct.pack('i', self.counter_pose)
            # alternative sending mode with same message lenght as reflector message
            n_ref = 10
            dummy_array = [0.0]*n_ref
            dummy_array[0] = x
            dummy_array[1] = y
            dummy_array[2] = yaw
            data_send_tc = struct.pack('i', self.counter_v_set)
            data_send_status = struct.pack('iii', -2, 0, 0)
            dataVector = f"{n_ref*2}f"
            data_send_list_dummy = struct.pack(dataVector, *dummy_array, *dummy_array)
            data_send = data_send_tc + data_send_status  + data_send_list_dummy  + data_send_list_dummy + data_send_tc
            # sending data via udp to mab
            self.sock.sendto(data_send, (self.UDP_IP_TARGET, self.UDP_PORT_TARGET))
            rospy.loginfo('udp_msg_sender: v_set msg sent')
        except:
            rospy.loginfo('udp_msg_sender: problem with sending data')
        

    def close_socket(self):
        # closing socket
        self.sock.close()
        rospy.loginfo('udp_msg_sender: socket closed')

if __name__ == '__main__':

    # setup listener
    autera_topic_listener = topic_listener()
    # hold program open
    rospy.spin()
    # close socket
    autera_topic_listener.close_socket()
