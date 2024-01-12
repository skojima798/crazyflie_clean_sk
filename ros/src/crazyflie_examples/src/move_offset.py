#!/usr/bin/env python3
import rospy
from std_msgs.msg import Empty
import numpy as np
from crazyflie_msgs.msg import PositionVelocityStateStamped
import subprocess

service= '/land'
command = ['rosservice', 'call', service] #definign the /land
count=0
    

#Defining the callback, which only runs after being told by the subscriber
def topic_callback(msg):
    rospy.loginfo("Sending new Coordinates {}".format(msg))
    global count #allows us to edit 

    desired_start_time = rospy.get_rostime() + rospy.Duration(9.0)  # Move to new coord after 2 seconds
    while rospy.get_rostime() < desired_start_time:
        rospy.sleep(1) 

    #Define new coordinate
    newref = PositionVelocityStateStamped()  # Create a new message object
    newref.header.stamp = rospy.Time.now()  # Set the header timestamp to the current time
    newref.state.x = 0.0  # Set the x coordinate
    newref.state.y = 3.0  # Set the y coordinate
    newref.state.z = 10.0  # Set the z coordinate
    newref.state.x_dot = 0.0  # Set the x velocity
    newref.state.y_dot = 0.0  # Set the y velocity
    newref.state.z_dot = 0.0  # Set the z velocity
    pub.publish(newref) #Publish to /ref the desired coordinate
    rospy.sleep(9) #let the drone stabilize
    subprocess.check_output(command)
    count=1
    print(count)
    

#Main loop which determines if something was published in takeoff
if __name__ == '__main__':
    rospy.init_node('takeoff_offset')  # Initialize the ROS node
    rospy.Subscriber('in_flight', Empty, topic_callback)  # Create a subscriber for the topic 'in_flight'
    pub = rospy.Publisher('ref', PositionVelocityStateStamped , queue_size=100)


    while not rospy.is_shutdown() and count==0:
        if 'in_flight' in rospy.get_published_topics():  # Check if 'in_flight' is in the published topics
            rospy.loginfo("'in_flight' is being published.") # Not actually being detected...?


        else:
            #rospy.loginfo("'in_flight' is NOT being published.")
            rospy.sleep(1)  # Delay the loop for 1 second

