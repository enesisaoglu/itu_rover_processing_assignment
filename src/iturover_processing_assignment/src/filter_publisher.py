#!/usr/bin/env python3

# @author yerenkl
# Date: 05/09/2023

import rospy
from std_msgs.msg import String
import random

class itu_rover_node:
    def __init__(self):
        rospy.init_node('itu_rover_node', anonymous=True)
        
        # Create a publisher for the '/filter' topic
        self.filter_publisher = rospy.Publisher('/filter', String, queue_size=10)
        
        self.filters = ["GRAY", "RESIZE_UP", "RESIZE_DOWN", "RGB"]
        self.rate = rospy.Rate(0.5)  
    def run(self):
        while not rospy.is_shutdown():
            random_filter = random.choice(self.filters)

            # Remove the used filter
            self.filters.remove(random_filter)      
            
            rospy.loginfo("Publishing: %s", random_filter)
            self.filter_publisher.publish(random_filter)
            
            # Check if all filters have been used and reset the list 
            if not self.filters:
                self.filters=["GRAY", "RESIZE_UP", "RESIZE_DOWN", "RGB"]
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = itu_rover_node()
        node.run()
    except rospy.ROSInterruptException:
        pass
