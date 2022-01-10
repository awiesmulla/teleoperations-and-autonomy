#!/usr/bin/env python3

import numpy as np
import serial
import math
import os
from sys import exit

class Readgps():
    def __init__(self):
        self.latitude = None
        self.longitude = None
        self.altitude = None
        #Constants used for conversion
        self.a = 6378137.0 #Radius of the earth
        self.odom_origin = np.array([0 , 0])
        #Counter for initializing GPS
        self.gps_counter = 0
        #Angular offset between odom and map frame
        #Multiplying rotation matrix to pose will rotate to axes anti-clockwise
        #We have to align x with East so rotate accordingly
        self.angle = np.deg2rad(-90)
        self.rotation_matrix = np.array([[np.cos(self.angle), np.sin(self.angle)],
                                         [-np.sin(self.angle), np.cos(self.angle)]])
    
    def read_gps(self):
        '''
        Read Lat, Long and altitude values
        '''
        ser = serial.Serial('/dev/ttyACM0', 4800, timeout=5)
        while True:
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

                        #print('lat, lon, num_satellites:', self.latitude,self.longitude,num_satellites)
                        # print('lon:', self.longitude)
                        # print('altitude(M):',self.altitude)
                        self.latitude = int(lat_deg) + float(lat_min)/60
                        self.longitude = int(lon_deg) + float(lon_min)/60
                        self.altitude = float(splitline[9])
                        #Convert to xyz
                        #print("odom_origin:",self.odom_origin)
                        self.conv_relative()
                        if self.gps_counter == 0:
                            self.odom_origin[0] = self.x
                            self.odom_origin[1] = self.y
                            self.x = 0
                            self.y = 0
                        else:
                            #Publish
                            print("====================================================")  
                        self.gps_counter += 1
                        #print("counter:",self.gps_counter)
            except Exception as e:
                print('error', e)
    

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
        
        if not self.gps_counter == 0:
            self.x = temp[0]
            self.y = temp[1]
        self.z = self.altitude
        print("self.x:",self.x)
        print("self.y:",self.y)

def main():

	Readgpsobj = Readgps()
	Readgpsobj.read_gps()

if __name__ == '__main__':
	main()