#!/usr/bin/env python3
# A simple ROS publisher node in Python

from logging import shutdown
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Publisher():
    
    def __init__(self):
        self.node_name = "simple_publisher"
        topic_name = "/cmd_vel"

        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.main_loop)
        rospy.init_node(self.node_name, anonymous=True)
        speed = 5
        self.rate = rospy.Rate(0.033333333 * speed) # hz
                
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        self.flipped = False
        self.started = False
        
        rospy.loginfo(f"The '{self.node_name}' node is active...")
        #self.rate.sleep()
        self.main_loop()

    def shutdownhook(self):
        print(f"Stopping the '{self.node_name}' node at: {rospy.get_time()}")
        self.ctrl_c = True

        vel_cmd = Twist()

        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.pub.publish(vel_cmd)

    def main_loop(self):

        while not self.ctrl_c:
            if self.flipped == False:
                if self.started == False:
                    self.started = True
                    self.flipped = True
                    vel_cmd = Twist()
                    vel_cmd.linear.x = 0.10472
                    vel_cmd.angular.z = -0.20943951
                    self.pub.publish(vel_cmd)
                    self.rate.sleep()
                else:
                    self.shutdownhook()
            else:
                self.flipped = False
                vel_cmd = Twist()
                vel_cmd.linear.x = 0.10472
                vel_cmd.angular.z = 0.20943951
                self.pub.publish(vel_cmd)
                self.rate.sleep()
                #self.shutdownhook
        

if __name__ == '__main__':
    publisher_instance = Publisher()
    try:
        publisher_instance.main_loop()
    except rospy.ROSInterruptException:
        pass