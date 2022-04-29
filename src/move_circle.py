#!/usr/bin/env python3
# A simple ROS publisher node in Python

import time
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Publisher():
    
    def __init__(self):
        self.node_name = "team_20_move_circle"
        topic_name = "/cmd_vel"

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        rospy.init_node(self.node_name, anonymous=True)
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")


    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")

        vel_cmd = Twist()

        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)

        self.ctrl_c = True

    def main_loop(self):

        flipped = 3

        while not self.ctrl_c:

            sign = 1

            if flipped % 2 == 0:
                sign = -1
            else:
                sign = 1

            if flipped == 0:
                self.shutdownhook()
            else:
                
                print ("flipped " + str(flipped))
                print ("sign" + str(sign))

                vel_cmd = Twist()

                vel_cmd.linear.x = 0.10472
                vel_cmd.angular.z = -0.20943951 * sign
                self.pub.publish(vel_cmd)
                flipped -= 1
                time.sleep(30)

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass