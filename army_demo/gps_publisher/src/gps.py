#!/usr/bin/env python
import rospy
import rospkg
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

import serial
import yaml
import numpy as np
import math
from sys import exit
import os

class Readgps():
    def __init__(self, map_origin):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        #Constants used for conversion
        self.a = 6378137.0 #Radius of the earth
        self.odom_origin = np.array([0 , 0])
        self.map_origin = map_origin
        #Counter for initializing GPS
        self.gps_counter = 0
        #List for calculating variance of GPS readings
        self.x_var_ls = []
        self.y_var_ls = []
        self.x_var = 0.0
        self.y_var = 0.0
        #Angular offset between odom and map frame
        #Multiplying rotation matrix to pose will rotate to axes anti-clockwise
        self.angle = np.deg2rad(0)
        self.rotation_matrix = np.array([[np.cos(self.angle), np.sin(self.angle)],
                                         [-np.sin(self.angle), np.cos(self.angle)]])
        #Rospy publisher
        rospy.init_node('gps_publisher')
        self.pub = rospy.Publisher('gps_odom',Odometry,queue_size=50)
        self.rate = rospy.Rate(1.0)
        # rospy.spin()
    
    def read_gps(self):
        '''
        Read Lat, Long and altitude values
        '''
        ser = serial.Serial('/dev/ttyUSB0', 4800, timeout=5) #TODO: Write udev rules for this
        while not rospy.is_shutdown():
            try:
                line = ser.readline()
                line = line.decode('utf-8')
                splitline = line.split(',')
                if splitline[0] == '$GPGGA':
                    #########################################################
                    if splitline[6] == '0':
                        print("Current GPS data invalid")
                        self.gps_counter = 0
                    #########################################################
                    else:
                        # print("GPS Initalization")
                        lat = splitline[2]
                        lat_deg = lat[:2] 
                        lat_min = lat[2:] 
                        latdirection = splitline[3]

                        lon = splitline[4]
                        lon_deg = lon[:3].lstrip("0")
                        lon_min = lon[3:]
                        londirection = splitline[5]

                        num_satellites = splitline[7]
                        
                        # self.latitude = lat_deg +' deg ' + lat_min +"'" + latdirection
                        # self.longitude = lon_deg + ' deg ' + lon_min + "'" + londirection

                        # self.altitude = splitline[9]

                        print('lat, lon, num_satellites:', self.latitude,self.longitude,num_satellites)
                        # print('lon:', self.longitude)
                        # print('altitude(M):',self.altitude)
                        self.latitude = int(lat_deg) + float(lat_min)/60
                        self.longitude = int(lon_deg) + float(lon_min)/60
                        self.altitude = float(splitline[9])
                        #Convert to xyz
                        print("odom_origin:",self.odom_origin)
                        self.conv_relative()
                        if self.gps_counter == 0:
                            self.odom_origin[0] = self.x
                            self.odom_origin[1] = self.y
                            self.x = 0
                            self.y = 0
                        elif self.gps_counter < 20:
                            # os.system('roslaunch copernicus_localization localization.launch')
                            self.x_var_ls.append(self.x)
                            self.y_var_ls.append(self.y)
                        elif self.gps_counter == 20:
                            #Calculate variance in the x and y directions
                            self.x_var = np.var(np.array(self.x_var_ls))
                            self.y_var = np.var(np.array(self.y_var_ls))
                            print("x_var:",self.x_var)
                            print("y_var:",self.y_var)
                        else:
                            #Publish
                            self.publish()
                            self.rate.sleep()  
                        self.gps_counter += 1
                        print("counter:",self.gps_counter)
            except Exception as e:
                print('error', e)
            # except KeyboardInterrupt:
                # exit()
    

    def conv_relative(self):
        '''
        Convert to relative coordinates using mercartor scale
        '''
        #For testing
        #self.latitude = 13.01
        #self.longitude = 77.57
        #self.altitude = 931
        #Constants used for conversion
        s = np.cos(self.latitude * np.pi/180)
        self.x = s * self.a * (np.pi*self.longitude/180)
        self.x = self.x - self.odom_origin[0]
        self.y = s * self.a * np.log(np.tan(np.pi*(90 + self.latitude)/360))
        self.y = self.y - self.odom_origin[1]
        temp = np.array([self.x, self.y])
        temp = np.dot(temp, self.rotation_matrix)
        self.x = temp[0]
        self.y = temp[1]
        self.z = self.altitude
        print("self.x:",self.x)
        print("self.y:",self.y)
    

    def publish(self):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'map'
        #Position
        # odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x 
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        # odom.pose.covariance[0] = self.x_var
        # odom.pose.covariance[7] = self.y_var
        odom.pose.covariance[0] = 100.0
        odom.pose.covariance[7] = 100.0
        #publish
        self.pub.publish(odom)
    
    def transform_to_global_frame(self):
        t = TransformStamped()
        t.transform.translation.x = self.x - self.map_origin[0]
        t.transform.translation.y = self.y - self.map_origin[1]
        tf.TransformBroadcaster.sendTransformMessage(t)

if __name__ == '__main__':
    rospack = rospkg.RosPack()
    config_file_path = rospack.get_path('gps_publisher') + '/config/config.yaml'
    print("config_file_path:",config_file_path)
    config_param = yaml.safe_load(open(config_file_path))
    map_origin = config_param['map_origin']
    Readgpsobj = Readgps(map_origin)
    Readgpsobj.read_gps()




