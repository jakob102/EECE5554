#!/usr/bin/env python3

import rospy
import math
import time
import serial
import rosbag

from std_msgs.msg import String
from vn_driver.msg import Vectornav

def set_vn_rate(vn_port_name ) :
    if vn_port_name. isOpen() == "False":
        vn_port_name.open ()
    vn_port_name.write(b'$VNWRG, 07, 40*XX\r\n')
    time.sleep (1)
    vn_port_name.write(b'$VNWRG,06, 14*XX\r\n')
    time.sleep (1)
    print("successfully changed rate to 40 Hz")
    vn_port_name.close ()

if __name__ == '__main__':
    rospy.init_node('Vnav', anonymous=True)
    v_msg = Vectornav()
    port = rospy.get_param('~port', '/dev/pts/9')
    serialPort = serial.Serial(port, baudrate = 115200, timeout = 1) #put in correct port HERE
    set_vn_rate(serialPort)
    
    pub = rospy.Publisher('/imu',Vectornav, queue_size=10)
    bag = rosbag.Bag('testing_lab3.bag','w')  
    
    #rate = rospy.Rate(10) # 10hz
try:    
    while not rospy.is_shutdown():
        # % rospy.get_time()
        
        #rate.sleep()
        #/dev/ttyUSB0
        ##                         port='/dev/pts/2',
        
        #port.write(b'$VNWRG,07,40*XX\r\n')
        #time.sleep(1)
        #port.write(b'$VNWRG,06,14*XX\r\n')

        if serialPort.isOpen() == False:
            serialPort.open ()


        new_lines = []

        for line in serialPort:
            v_msg.vnymr_read = str(line.decode('utf-8').strip())
            items = v_msg.vnymr_read.split(",")
            label=items[0]
            
            if label=="$VNYMR":
                v_msg.header.frame_id = 'imu1_frame'
                #items = v_msg.vnymr_read.split(",")	#split the v_msg.vnymr_read into items by every commma sign
                yaw = float(items[1])*(math.pi/180)           #rad (z) (psi)
                pitch = float(items[2])*(math.pi/180)         #rad (y) (phi)
                roll = float(items[3])*(math.pi/180)          #rad (x) (theta)
                mag_x = float(items[4])         #unitless (magnitude)
                mag_y = float(items[5])         #unitless (magnitude)
                mag_z = float(items[6])         #unitless (magnitude)
                accel_x = float(items[7])       #m/s^2
                accel_y = float(items[8])       #m/s^2
                accel_z = float(items[9])      #m/s^2
                gyro_x = float(items[10])       #rad/s
                gyro_y = float(items[11])       #rad/s
                gyro_z = (items[12]).split('*')[0] #splitting ####*6F and splitting into two, then takes first value
                gyro_z = float(gyro_z)     #rad/s
                

        ### this is my function to convert the above euler angles to quaternions      
                v_msg.imu.orientation.w = math.cos(yaw/2)*math.cos(pitch/2)*math.cos(roll/2)+math.sin(yaw/2)*math.sin(pitch/2)*math.sin(roll/2)
                v_msg.imu.orientation.x = math.cos(yaw/2)*math.cos(roll/2)*math.sin(pitch/2)-math.sin(yaw/2)*math.cos(pitch/2)*math.sin(roll/2)
                v_msg.imu.orientation.y = math.cos(yaw/2)*math.sin(roll/2)*math.cos(pitch/2)+math.sin(yaw/2)*math.sin(pitch/2)*math.cos(roll/2)
                v_msg.imu.orientation.z = math.sin(yaw/2)*math.cos(roll/2)*math.cos(pitch/2)-math.cos(yaw/2)*math.sin(pitch/2)*math.sin(roll/2)

                v_msg.imu.angular_velocity.x = gyro_x
                v_msg.imu.angular_velocity.y = gyro_y
                v_msg.imu.angular_velocity.z = gyro_z

                v_msg.imu.linear_acceleration.x = accel_x
                v_msg.imu.linear_acceleration.y = accel_y
                v_msg.imu.linear_acceleration.z = accel_z

                v_msg.mag_field.magnetic_field.x = mag_x
                v_msg.mag_field.magnetic_field.y = mag_y
                v_msg.mag_field.magnetic_field.z = mag_z


        ########## this is my function to convert the above UTC time to epoch seconds for ROS
                
                stime =int(time.time())
                ntime = int(time.time_ns())
                v_msg.header.stamp.secs = stime
                ntime = int(ntime-stime*1e9)
                v_msg.header.stamp.nsecs = ntime

                v_msg.imu.header.stamp.secs = stime
                v_msg.mag_field.header.stamp.secs = stime

                v_msg.imu.header.stamp.nsecs = ntime
                v_msg.mag_field.header.stamp.nsecs = ntime

                print(v_msg)
                #new_lines.append(new_line) #adds the new v_msg.vnymr_read to a list

                pub.publish(v_msg)
                rospy.loginfo(v_msg)        #this v_msg.vnymr_read prints to terminal / ROS work space, just a print statement
                bag.write('/imu', v_msg)
                    
                if rospy.is_shutdown():
                
                    serialPort.close()

finally:
    bag.close()