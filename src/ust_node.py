#!/usr/bin/env python
from serial import Serial
import time
from collections import namedtuple
import rospy
from sensor_msgs.msg import LaserScan
import tf
import os
import sys

class UST:
    
    Command = namedtuple('Command',['command', 'answer_expected', 'answer'])

    START_RANGING = Command('#GT15466', False, '')
    STOP_RANGING = Command('#ST5297', True, '#ST00A845')
    ID = Command('#IN0D54', True, '')
    ID2 = Command('#CLC2DD', True, '')

    PLOP = Command('#GR0EEE1', True, '')
    
    MESURE_LENGHT = 4359

    def __init__(self, port, baudrate=115200):
        self.ser = Serial(port, baudrate)
        self.stop_ranging()
        self.data_prev = ''

    def send_command(self, command, timeout=2):
        self.ser.write(command.command.encode()+b'\n')    # writes command to LIDAR
        self.ser.reset_input_buffer()
        data = b''
        start_time = time.time()
        if command.answer_expected and command.answer != '':    #precise answer expected, search for it !
            while time.time() - start_time < timeout:
                if self.ser.in_waiting:
                    data+=self.ser.read()
                    data = data.split(b'\n')[-1]
                    if command.answer.encode() in data:
                        break
            return data
        elif command.answer_expected:   # answer expected but be don't known which : return the first one (until \n)
            while time.time() - start_time < timeout:
                if self.ser.in_waiting:
                    data+=self.ser.read()
                    if b'\n' in data:
                        data = data.split(b'\n')[0]
                        break
            return data
        else:
            return b''

    def stop_ranging(self):
        self.send_command(self.STOP_RANGING)

    def start_ranging(self):
        self.stop_ranging()
        self.send_command(self.START_RANGING)

    def stop(self):
        self.stop_ranging()
    
    def get_measures(self):
        """
        returns measures under the form (timestamp, [(distance, quality), ...])
        timestamp : time in seconds since the LIDAR startup
        distance range : 0 - 65635
        valur range : 0 - ??? (65635 max)
        eg: (102.123456, [(552, 1244), (646, 1216), (676, 1270), ...])
        """
        raw_bytes = self.ser.read(ust.MESURE_LENGHT)
        raw_bytes_str = str(raw_bytes)

        data = raw_bytes_str[26:]
        
        if data.find("#") > 1:
            print("true")
            data = self.data_prev
            ust.send_command(ust.ID)
            
        measurements = [(int(data[i:i+4],16), int(data[i+4:i+8],16))  for i in range(0, len(data)-8, 8)]
        self.data_prev = data
        return (measurements)
    
    def get_data(self, nb_bytes):
        data = self.ser.read(nb_bytes)
        data = data.split(b':')
        data = data[3]
        aa = [(int(data[i:i+4],16), int(data[i+4:i+8],16))  for i in range(0, len(data)-8, 8)]
    

if __name__ == "__main__":
    rospy.init_node('urg_node')
    
    topic_name = str(rospy.get_param('~topic_name', "/scan"))
    pub = rospy.Publisher(topic_name, LaserScan, queue_size=10)
    rate = rospy.Rate(40)
    scan_msg = LaserScan()
    br = tf.TransformBroadcaster()
    
    frame = str(rospy.get_param('~frame_id', "laser_link"))
    offset = float(rospy.get_param('~offset', 0))
    max_angle = float(rospy.get_param('~max_angle', 1.178097245))
    min_angle = float(rospy.get_param('~max_angle', -1.178097245))
    port = str(rospy.get_param('~dev', "/dev/ttyACM0"))
    
    link_tf_id = str(rospy.get_param('~link_tf_id', "world"))
    link_tf_param = str(rospy.get_param('~link_tf_param', "0, 0, 0, 0, 0, 0"))
    link_tf_param = link_tf_param.split(',')
    ust = UST(port)
    try:
        ret = ust.send_command(ust.ID)
        #print(ret)
        ust.start_ranging()
        while not rospy.is_shutdown():
            data = ust.get_measures()
                
            if len(data) == 541 :
                #print("ok at {:.06f} s".format(data[0]))
                laser_data = data
                distance_list = []
                intensity_list = []
                for i in range(541):
                    scan_data = laser_data[i]
                    distance_list.append(scan_data[0] * 0.001)
                    intensity_list.append(scan_data[1])
                    
                br.sendTransform((float(link_tf_param[0]), float(link_tf_param[1]), float(link_tf_param[2])),
                     tf.transformations.quaternion_from_euler(float(link_tf_param[3]), float(link_tf_param[4]), float(link_tf_param[5])),
                     rospy.Time.now(),
                     frame,
                     link_tf_id)
                     
                scan_msg.ranges = distance_list
                scan_msg.intensities = intensity_list
                scan_msg.header.stamp = rospy.Time.now()
                scan_msg.header.frame_id = frame
                scan_msg.angle_min = -1.178097245 + offset;
                scan_msg.angle_max = 1.178097245 + offset;
                scan_msg.angle_increment = 0.008726646 #0,004355258
                scan_msg.scan_time = 1 / 40
                scan_msg.time_increment = (1 / 40) / 540.
                
                scan_msg.range_min = 0.0;
                scan_msg.range_max = 5.0;
                
                pub.publish(scan_msg)
                rate.sleep()
                
    finally:
        ust.stop()