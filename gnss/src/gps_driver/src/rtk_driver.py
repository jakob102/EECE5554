#!/usr/bin/env python3

import rospy
import utm
import math
import time
import calendar
import serial
import rosbag

from std_msgs.msg import String
from gps_driver.msg import Customrtk

if __name__ == '__main__':
    
    rospy.init_node('rtk_gnss', anonymous=True)
    c_msg = Customrtk()
    pub = rospy.Publisher('gps',Customrtk, queue_size=10)
    bag = rosbag.Bag('walkingRTK.bag','w')  
    
    #rate = rospy.Rate(10) # 10hz
try:    
    while not rospy.is_shutdown():
        # % rospy.get_time()
        
        #rate.sleep()
        #/dev/ttyUSB0
        ##                         port='/dev/pts/2',
        port = rospy.get_param('~port', '/dev/pts/8')
        serialPort = serial.Serial(port, baudrate = 4800, timeout = 1) #put in correct port HERE

    
        new_lines = []

        for line in serialPort:
            c_msg.gngga_read = str(line)
            items = c_msg.gngga_read.split(",")
            label=items[0]
            if label=="b'$GNGGA":
                c_msg.header.frame_id = 'GPS1_Frame'
                items = c_msg.gngga_read.split(",")	#split the c_msg.gngga_read into items by every commma sign
                UTC = float(items[1])
                latitude = float(items[2]) #in DD.mmmm 
                latdir = str(items[3])
                longitude = float(items[4]) #in DD.mmmm 
                londir = str(items[5])
                c_msg.hdop = float(items[8])
                c_msg.altitude = float(items[9])
                c_msg.fix_quality = int(items[6])

        ### this if my function to convert the above degree minute values to degree decimal I am not sure how to separate it out yet      
                if 'S' in latdir:
                    latitude = latitude/-100
                else:
                    latitude = latitude/100
                if 'W' in londir:
                    longitude = longitude/-100
                else:
                    longitude = longitude/100
                lat_degree = math.trunc(latitude)
                lon_degree = math.trunc(longitude)
                lat_decimal = (latitude-lat_degree)/0.6
                lon_decimal = (longitude-lon_degree)/0.6
                c_msg.latitude = float(lat_degree+lat_decimal)
                c_msg.longitude= float(lon_degree+lon_decimal)

        ########## this is my function to convert the above UTC time to epoch seconds for ROS

                time1 = time.gmtime() #gets the current utc date and time
                time1 = time1.tm_year,time1.tm_mon,time1.tm_mday,0,0,0 #sets the date and time at the beginning of today
                es_beginning_of_day = calendar.timegm(time1) #converts beginning of today to epoch seconds

                UTC_h = int(UTC/10000)
                UTC = UTC-UTC_h*10000
                UTC_m = int(UTC/100)
                UTC = UTC-UTC_m*100
                UTC_s = int(UTC)
                UTC = UTC-UTC_s
                c_msg.header.stamp.nsecs = int(UTC*1e9)

                time_gps = 1970,1,1,UTC_h,UTC_m,UTC_s
                es_now = calendar.timegm(time_gps) #seconds since midnight

                es_time_of_gps_coord = es_beginning_of_day+es_now
                c_msg.header.stamp.secs = int(es_time_of_gps_coord)

        ############

                latitude = c_msg.latitude
                longitude = c_msg.longitude
                a,b,c,d = utm.from_latlon(latitude, longitude) #input the latitude and longitude into the converter and we get 4 new values assigned to abcd
                new_line = str(a) + " " + str(b) + " " + str(c) + " " +  d	# saves the values in a c_msg.gngga_read
                c_msg.utm_easting = float(a)
                c_msg.utm_northing = float(b)
                c_msg.zone = c
                c_msg.letter = d

                new_lines.append(new_line) #adds the new c_msg.gngga_read to a list

                pub.publish(c_msg)
                rospy.loginfo(c_msg)        #this c_msg.gngga_read prints to terminal / ROS work space, just a print statement
                bag.write('gps', c_msg)
                    
                if rospy.is_shutdown():
                
                    serialPort.close()

finally:
    bag.close()