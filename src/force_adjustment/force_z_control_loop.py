import rospy
from force_adjustment.msg import ForceTorque
from iiwa_msgs.msg import CartesianPose
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Time
import numpy as np
from scipy.spatial.transform import Rotation as R
import setup_control_mode


class RobotInstance:

    def __init__(self):

        self.current_pose_subscriber = rospy.Subscriber("/iiwa/state/CartesianPose", CartesianPose, self.current_pose_callback)
        self.force_torque_subscriber = rospy.Subscriber("ATI_force_torque", ForceTorque, self.fz_callback)
        self.new_pose_publisher = rospy.Publisher("/iiwa/command/CartesianPose", PoseStamped, queue_size=5)

        self.current_pose = None
        self.start_pose = None
        # Should be a PoseStamped typed message
        self.goal_pose = None
        
        self.force_torque = None
        self.upper_threshold = 10
        self.lower_threshold = 5
        self.desired_force = None
        self.step_size = 0.0005
        

    def current_pose_callback(self, data):
        self.current_pose = data

    def fz_callback(self, data):
        self.force_torque = data

    def publish_pose(self, pose):
        self.new_pose_publisher.publish(pose)


def generate_base_tool_transformation_matrix(robot_instance_object):
    translation_matrix = np.array([robot_instance_object.current_pose.poseStamped.pose.position.x,
                                   robot_instance_object.current_pose.poseStamped.pose.position.y,
                                   robot_instance_object.current_pose.poseStamped.pose.position.z])

    rotation = R.from_quat([robot_instance_object.current_pose.poseStamped.pose.orientation.x, 
                            robot_instance_object.current_pose.poseStamped.pose.orientation.y, 
                            robot_instance_object.current_pose.poseStamped.pose.orientation.z, 
                            robot_instance_object.current_pose.poseStamped.pose.orientation.w])
    rotation_matrix = rotation.as_dcm()

    transformation_matrix = np.array([[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], translation_matrix[0]],
                                      [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], translation_matrix[1]],
                                      [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], translation_matrix[2]],
                                      [0, 0, 0, 1]])

    return transformation_matrix


def transform_tool_to_base(transformation_matrix, step_size):
    target_coordinates = np.array([0, 0, step_size, 1])
    return np.matmul(transformation_matrix, target_coordinates)


# Creates goal pose message from input: (0,1,2)=(X,Y,Z) and distance in meters
def create_goal_pose_message(robot_instance_object, axis, distance):
    goal_pose = PoseStamped()
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.header.frame_id = 'iiwa_link_0'

    goal_pose.pose.position.x = robot_instance_object.current_pose.poseStamped.pose.position.x
    goal_pose.pose.position.y = robot_instance_object.current_pose.poseStamped.pose.position.y 
    goal_pose.pose.position.z = robot_instance_object.current_pose.poseStamped.pose.position.z

    goal_pose.pose.orientation.x = robot_instance_object.current_pose.poseStamped.pose.orientation.x
    goal_pose.pose.orientation.y = robot_instance_object.current_pose.poseStamped.pose.orientation.y
    goal_pose.pose.orientation.z = robot_instance_object.current_pose.poseStamped.pose.orientation.z
    goal_pose.pose.orientation.w = robot_instance_object.current_pose.poseStamped.pose.orientation.w

    if(axis == 0):
        goal_pose.pose.position.x += distance

    if(axis == 1):
        goal_pose.pose.position.y += distance

    if(axis == 2):
        goal_pose.pose.position.z += distance
    return goal_pose
    

def correct_force(robot_instance_object, low):
    if low:
        print("increasing force...")
        transformed_coordinates = transform_tool_to_base(generate_base_tool_transformation_matrix(robot_instance_object), robot_instance_object.step_size)

        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = 'iiwa_link_0'
        new_pose.pose.position.x = transformed_coordinates[0]
        new_pose.pose.position.y = transformed_coordinates[1]
        new_pose.pose.position.z = transformed_coordinates[2]
        new_pose.pose.orientation = robot_instance_object.current_pose.poseStamped.pose.orientation
        
        print("publishing new pose...")
        robot_instance_object.publish_pose(new_pose)
        rospy.sleep(1)
    else: 
        print("decreasing force")
        transformed_coordinates = transform_tool_to_base(generate_base_tool_transformation_matrix(robot_instance_object), -robot_instance_object.step_size)

        new_pose = PoseStamped()
        new_pose.header.stamp = rospy.Time.now()
        new_pose.header.frame_id = 'iiwa_link_0'
        new_pose.pose.position.x = transformed_coordinates[0]
        new_pose.pose.position.y = transformed_coordinates[1]
        new_pose.pose.position.z = transformed_coordinates[2]
        new_pose.pose.orientation = robot_instance_object.current_pose.poseStamped.pose.orientation
        
        print("publishing new pose...")
        robot_instance_object.publish_pose(new_pose)
        rospy.sleep(1)


def check_force(robot_instance_object):
    if robot_instance_object.force_torque is not None and robot_instance_object.current_pose is not None:
            if abs(robot_instance_object.force_torque.fz) < robot_instance_object.lower_threshold:
                correct_force(robot_instance_object, True)
                
            if abs(robot_instance_object.force_torque.fz) > robot_instance_object.upper_threshold:
                correct_force(robot_instance_object, False)
            else:
                return


def start_control_loop(robot_instance_object):
    while True:
        if robot_instance_object.force_torque is not None and robot_instance_object.current_pose is not None:
            if abs(robot_instance_object.force_torque.fz) < robot_instance_object.lower_threshold:
                # correct_force(robot_instance_object, True)
                check_force(robot_instance_object)
                
            if abs(robot_instance_object.force_torque.fz) > robot_instance_object.upper_threshold:
                # correct_force(robot_instance_object, False)
                check_force(robot_instance_object)

            else: 
                robot_instance_object.goal_pose.pose.position.z = robot_instance_object.current_pose.poseStamped.pose.position.z
                robot_instance_object.publish_pose(robot_instance_object.goal_pose)
        else: 
            continue


if __name__ == "__main__":
    rospy.init_node("fz_control_loop", anonymous=True)
    setup_control_mode.set_cartesian_impedance_control_mode([3000, 3000, 1000, 300, 300, 300], [0.7, 0.7, 0.7, 0.7, 0.7, 0.7], 2000, 0.7, 1000, 1000)
    robot_instance = RobotInstance()
    robot_instance.start_pose = rospy.wait_for_message("/iiwa/state/CartesianPose", CartesianPose, timeout=5)

    # Create goal pose
    create_goal_pose_message(robot_instance, 1, 0.05)
    # robot_instance.goal_pose = PoseStamped()
    # robot_instance.goal_pose.header.stamp = rospy.Time.now()
    # robot_instance.goal_pose.header.frame_id = 'iiwa_link_0'
    # robot_instance.goal_pose.pose.position.x = robot_instance.current_pose.poseStamped.pose.position.x - 0.05
    # robot_instance.goal_pose.pose.position.y = robot_instance.current_pose.poseStamped.pose.position.y 
    # robot_instance.goal_pose.pose.position.z = robot_instance.current_pose.poseStamped.pose.position.z 
    # robot_instance.goal_pose.pose.orientation.x = robot_instance.current_pose.poseStamped.pose.orientation.x
    # robot_instance.goal_pose.pose.orientation.y = robot_instance.current_pose.poseStamped.pose.orientation.y
    # robot_instance.goal_pose.pose.orientation.z = robot_instance.current_pose.poseStamped.pose.orientation.z
    # robot_instance.goal_pose.pose.orientation.w = robot_instance.current_pose.poseStamped.pose.orientation.w

    start_control_loop(robot_instance)