import rospy
from force_adjustment.msg import ForceTorque

def newton_kg_converter(data):
    fz = data.fz
    newton_in_kg = fz * 0.1019716213
    print(newton_in_kg)

if __name__ == '__main__':
    rospy.init_node("newton_kg_converter")
    rospy.Subscriber("ATI_force_torque", ForceTorque, newton_kg_converter)
    rospy.spin()
