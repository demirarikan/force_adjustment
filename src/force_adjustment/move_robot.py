#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped 

# Creates and publishes the starting position for the probe placement
def reset():
    pub = rospy.Publisher('/iiwa/command/CartesianPose', PoseStamped, queue_size=5)
    newPose = PoseStamped()
    newPose.header.stamp = rospy.Time.now()
    newPose.header.frame_id = 'iiwa_link_0'
    newPose.pose.position.x = 0.614237459705
    newPose.pose.position.y = -0.0409711049467
    newPose.pose.position.z = 0.147538457831
    newPose.pose.orientation.x = 0.000466301293374
    newPose.pose.orientation.y = 0.999783635139
    newPose.pose.orientation.z = -0.00155468122314
    newPose.pose.orientation.w = 0.0207368925045

    # TESTING POSITION: WITH SCALE, NO PHANTOM
    # position: 
    #   x: 0.614237459705
    #   y: -0.0409711049467
    #   z: 0.147538457831
    # orientation: 
    #   x: 0.000466301293374
    #   y: 0.999783635139
    #   z: -0.00155468122314
    #   w: 0.0207368925045



    rospy.sleep(0.3)
    pub.publish(newPose)

if __name__ == '__main__':
    rospy.init_node('resetPos', anonymous=True)
    rospy.sleep(0.3)
    reset()

