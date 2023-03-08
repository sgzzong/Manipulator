#! /usr/bin/env python3
import rospy                                          
from std_msgs.msg import String          
def callback(msg):                                                                                   
    print(msg.data)      
rospy.init_node('topic_subscriber')                   
sub = rospy.Subscriber('/state', String, callback)
pub = rospy.Publisher('/state',String, queue_size=1)

for i in range()

rospy.spin() 