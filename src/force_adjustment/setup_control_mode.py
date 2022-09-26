#!/usr/bin/env python
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest, ConfigureControlModeResponse
from iiwa_msgs.msg import ControlMode, DesiredForceControlMode, CartesianControlModeLimits, CartesianQuantity, JointQuantity
import math
import sys
import rospy


def cartesian_quantity_from_float(value):
    quantity = CartesianQuantity()
    for attr in quantity.__slots__:
        setattr(quantity, attr, value)
    return quantity


def configure_control_mode(request):
    rospy.wait_for_service('/iiwa/configuration/ConfigureControlMode') # FOR LUKE SET CHANGE TO: /luke/configuration/ConfigureControlMode
    print('Force mode enter')

    try:

        configure_control_mode_service = rospy.ServiceProxy('/iiwa/configuration/ConfigureControlMode', ConfigureControlMode)
        response = configure_control_mode_service(request)
        return response
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def set_position_control_mode():
    request = ConfigureControlModeRequest()
    request.control_mode = ControlMode.POSITION_CONTROL
    configure_control_mode(request)


# Use desired_force = 2N, desired_stiffness=500
def set_force_mode(cartesian_dof, desired_force, desired_stiffness, max_deviation_pos, max_deviation_rotation_in_deg):
    request = ConfigureControlModeRequest()
    request.control_mode = ControlMode.DESIRED_FORCE
    request.desired_force.cartesian_dof = cartesian_dof
    request.desired_force.desired_force = desired_force
    request.desired_force.desired_stiffness = desired_stiffness
    request.limits.max_control_force_stop = False
    request.limits.max_control_force = cartesian_quantity_from_float(-1)
    request.limits.max_cartesian_velocity = cartesian_quantity_from_float(-1)

    max_dev = request.limits.max_path_deviation
    max_dev.x = max_dev.y = max_dev.z = max_deviation_pos
    max_dev.a = max_dev.b = max_dev.c = math.radians(max_deviation_rotation_in_deg)
    configure_control_mode(request)
    print('Force mode set')

# Stiffness array is 7 element array with stiffness for each joint, same for damping
def set_joint_impedance_mode(joint_stiffness_array, joint_damping_array, max_deviation_pos, max_deviation_rotation_in_deg):
    request = ConfigureControlModeRequest()
    request.control_mode = ControlMode.JOINT_IMPEDANCE

    joint_stiffness = JointQuantity()
    joint_stiffness.a1 = joint_stiffness_array[0]
    joint_stiffness.a2 = joint_stiffness_array[1]
    joint_stiffness.a3 = joint_stiffness_array[2]
    joint_stiffness.a4 = joint_stiffness_array[3]
    joint_stiffness.a5 = joint_stiffness_array[4]
    joint_stiffness.a6 = joint_stiffness_array[5]
    joint_stiffness.a7 = joint_stiffness_array[6]

    request.joint_impedance.joint_stiffness = joint_stiffness

    joint_damping = JointQuantity()
    joint_damping.a1 = joint_damping_array[0]
    joint_damping.a2 = joint_damping_array[1]
    joint_damping.a3 = joint_damping_array[2]
    joint_damping.a4 = joint_damping_array[3]
    joint_damping.a5 = joint_damping_array[4]
    joint_damping.a6 = joint_damping_array[5]
    joint_damping.a7 = joint_damping_array[6]

    request.joint_impedance.joint_damping = joint_damping

    request.limits.max_control_force_stop = False
    request.limits.max_control_force = cartesian_quantity_from_float(-1)
    request.limits.max_cartesian_velocity = cartesian_quantity_from_float(-1)

    max_dev = request.limits.max_path_deviation
    max_dev.x = max_dev.y = max_dev.z = max_deviation_pos
    max_dev.a = max_dev.b = max_dev.c = math.radians(max_deviation_rotation_in_deg)
    print("configuring control mode")
    configure_control_mode(request)
    print('Joint impedance mode set')

# Dof:(1,2,3) for (X,Y,Z), Frequency and amplitude of the sine curve, stuffness of the robot
def set_sine_pattern_control_mode(cartesian_dof, frequency, amplitude, stiffness, max_deviation_pos, max_deviation_rotation_in_deg):
    request = ConfigureControlModeRequest()
    request.control_mode = ControlMode.SINE_PATTERN
    
    request.sine_pattern.cartesian_dof = cartesian_dof
    request.sine_pattern.frequency = frequency
    request.sine_pattern.amplitude = amplitude
    request.sine_pattern.stiffness = stiffness

    request.limits.max_control_force_stop = False
    request.limits.max_control_force = cartesian_quantity_from_float(-1)
    request.limits.max_cartesian_velocity = cartesian_quantity_from_float(-1)

    max_dev = request.limits.max_path_deviation
    max_dev.x = max_dev.y = max_dev.z = max_deviation_pos
    max_dev.a = max_dev.b = max_dev.c = math.radians(max_deviation_rotation_in_deg)
    configure_control_mode(request)
    print('Sine pattern mode set')


def set_cartesian_impedance_control_mode(cartesian_stiffness_array, cartesian_damping_array, nullspace_stiffness, nullspace_damping, max_deviation_pos, max_deviation_rotation_in_deg):
    request = ConfigureControlModeRequest()
    request.control_mode = ControlMode.SINE_PATTERN
    
    cartesian_stiffness = CartesianQuantity()
    cartesian_stiffness.x = cartesian_stiffness_array[0]
    cartesian_stiffness.y = cartesian_stiffness_array[1]
    cartesian_stiffness.z = cartesian_stiffness_array[2]
    cartesian_stiffness.a = cartesian_stiffness_array[3]
    cartesian_stiffness.b = cartesian_stiffness_array[4]
    cartesian_stiffness.c = cartesian_stiffness_array[5]
    request.cartesian_impedance.cartesian_stiffness = cartesian_stiffness

    cartesian_damping = CartesianQuantity()
    cartesian_damping.x = cartesian_damping_array[0]
    cartesian_damping.y = cartesian_damping_array[1]
    cartesian_damping.z = cartesian_damping_array[2]
    cartesian_damping.a = cartesian_damping_array[3]
    cartesian_damping.b = cartesian_damping_array[4]
    cartesian_damping.c = cartesian_damping_array[5]
    request.cartesian_impedance.cartesian_damping = cartesian_damping

    request.cartesian_impedance.nullspace_stiffness = nullspace_stiffness
    request.cartesian_impedance.nullspace_damping = nullspace_damping

    request.limits.max_control_force_stop = False
    request.limits.max_control_force = cartesian_quantity_from_float(-1)
    request.limits.max_cartesian_velocity = cartesian_quantity_from_float(-1)

    max_dev = request.limits.max_path_deviation
    max_dev.x = max_dev.y = max_dev.z = max_deviation_pos
    max_dev.a = max_dev.b = max_dev.c = math.radians(max_deviation_rotation_in_deg)
    configure_control_mode(request)
    print('Cartesian impedance control mode set')