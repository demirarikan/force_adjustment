import rospy
from geometry_msgs.msg import PoseStamped 
from iiwa_msgs.msg import CartesianPose
import setup_control_mode as scm

# Creates and publishes the starting position for the probe placement
def reset():
    newPose = PoseStamped()
    newPose.header.stamp = rospy.Time.now()
    newPose.header.frame_id = 'iiwa_link_0'
    newPose.pose.position.x = 0.0
    newPose.pose.position.y = -0.64
    newPose.pose.position.z = 0.35
    newPose.pose.orientation.x = 0.69831685594
    newPose.pose.orientation.y = 0.715637981892
    newPose.pose.orientation.z = 0.0108277900385
    newPose.pose.orientation.w = -0.00993151843735

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

def start_scan(stiffness, desired_force):
    scm.set_force_mode(3, desired_force, stiffness, 1000, 1000)
    rospy.sleep(0.5)
    curr_pose = rospy.wait_for_message("/iiwa/state/CartesianPose", CartesianPose)
    goal_pose = curr_pose.poseStamped 
    goal_pose.pose.position.x = goal_pose.pose.position.x - 0.22
    pub.publish(goal_pose)

if __name__ == '__main__':
    rospy.init_node('trajectoryTest', anonymous=True)
    pub = rospy.Publisher('/iiwa/command/CartesianPoseLin', PoseStamped, queue_size=5)
    start_scan(500, 5)

