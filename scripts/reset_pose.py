import rospy
from geometry_msgs.msg import PoseStamped 
import setup_control_mode as scm

def reset():
    newPose = PoseStamped()
    newPose.header.stamp = rospy.Time.now()
    newPose.header.frame_id = 'iiwa_link_0'
    newPose.pose.position.x = -0.226176468767
    newPose.pose.position.y = -0.616341736704
    newPose.pose.position.z =  0.332170722748
    newPose.pose.orientation.x = 0.762733817101
    newPose.pose.orientation.y = 0.581712549371
    newPose.pose.orientation.z =  0.243759055388
    newPose.pose.orientation.w = 0.142930676393

# position: 
#       x: 0.0648856273252
#       y: -0.659623317372
#       z: 0.428346394902
#     orientation: 
#       x: -0.0325478015234
#       y: 0.999158620834
#       z: -0.0116334137852
    # #   w: -0.0220763286296

    # position: 
    #   x: -0.226176468767
    #   y: -0.616341736704
    #   z: 0.332170722748
    # orientation: 
    #   x: 0.762733817101
    #   y: 0.581712549371
    #   z: 0.243759055388
    #   w: 0.142930676393




    rospy.sleep(0.3)
    pub.publish(newPose)

if __name__ == '__main__':
    rospy.init_node('resetPos', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPoseLin', PoseStamped, queue_size=5)
    rospy.sleep(0.3)
    reset()