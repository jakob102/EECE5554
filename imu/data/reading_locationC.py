#!/usr/bin/env python3

import rospy

import rosbag

from std_msgs.msg import String

########################
#baglist = []

rospy.init_node("hourbag", anonymous=True)
pub = rospy.Publisher('/location',String, queue_size=10)

bag1 = rosbag.Bag('/home/jakob102/gnss/src/vn_driver/python/LocationC.bag')
#for topic, msg, t in bag1.read_messages(topics=['/vectornav']):
#        a = (msg.data)
#        #baglist.append(a)
#        pub.publish(str(a))
#        rospy.loginfo(str(a)) 
#bag1.close()
#print(baglist)


with open('/home/jakob102/gnss/src/vn_driver/python/file2.xlsx', 'w') as file:
    for topic, msg, t in bag1.read_messages(topics=['/vectornav']):
        a = msg.data
        file.write(a)  # Write each string to a new line in the file
        #rospy.loginfo(a)

bag1.close()
