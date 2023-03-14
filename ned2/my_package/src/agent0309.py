#! /usr/bin/env python3
import rospy                                          
from std_msgs.msg import String
import geometry_msgs.msg
go = ['action 20 0 0 0 0 0', 'action 0 0 0 0 0 0', 'action 30 0 0 0 0 0']
class agent():
    def __init__(self):
        rospy.init_node('agent')                   
        self.sub = rospy.Subscriber('/state', String, self.callback, queue_size=1)
        self.pub = rospy.Publisher('/action',String, queue_size=1)

    def callback(self,msg):                                                                                  
        print(msg.data)

    def send(self):
        global go
        while(1):
            self.pub.publish(go[1]) 

def main():
    global angle 
    rl = agent()
    # ned2_contoller.action(angle[0])
    # ned2_contoller.get_joint()
    # #ned2_contoller.reward(ned2_contoller.target[0])
    # ned2_contoller.send(ned2_contoller.joint_angle, ned2_contoller.reward)
    rl.send()
    print(rl.flag)
if __name__ == '__main__':
    main()