import sys
import atiiaftt 
import pyigtl
import rospy
import numpy as np

from force_adjustment.msg import ForceTorque
from std_msgs.msg import Bool

class ForceSensorClient():

    def __init__(self):
        self.open_igt_client = None
        self.sensor = None
        self.reset_flag = False
        self.force_sensor_reset_sub = rospy.Subscriber("force_sensor_reset", Bool, self.force_sensor_reset_sub_callback)

    def force_sensor_reset_sub_callback(self, data):
        self.reset_flag = data
    
    def get_average_voltage_readings(self):
        """
        Calculates the average voltage readings over 100 samples.
        """
        print("Calculating average readings...")
        sum = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        divisor = 100.0
        for i in range(100):
            voltages = self.open_igt_client.wait_for_message("voltage_server", timeout=5)
            if voltages == None:
                print("TIMEOUT REACHED!\nConnection could not be established, please restart the server.")
                self.open_igt_client.stop()
                sys.exit()
            sum += openIGTLmessage_to_list(voltages)

        print("Done!")
        average_values = np.ndarray.tolist(sum/divisor)
        return average_values

    def convert_voltage_to_force(self, message):
        """
        Converts voltage values to force values using the sensor classes convert function.
        """
        voltages = openIGTLmessage_to_list(message)
        forces = self.sensor.convertToFt(voltages)
        return forces

def openIGTLmessage_to_list(message):
    """
    Converts Open IGT link string messages with voltage data to python float list.
    """
    # Get string body of message
    voltage_vals_str = message.string
    # Remove the brackets in the beginning and end, then convert it to a list
    voltage_vals_list = voltage_vals_str[1:-1].rsplit(" ")
    # Remove empty strings from list
    while "" in voltage_vals_list:
        voltage_vals_list.remove('')
    # Convert all elements to float
    voltage_vals_list = [float(x) for x in voltage_vals_list]
    return voltage_vals_list


def create_force_torque_message(force_torque_values):
    """
    Creates a ForceTorque message from given force torque value 6 element list. 
    """
    force_torque_message = ForceTorque()
    force_torque_message.fx = force_torque_values[0]
    force_torque_message.fy = force_torque_values[1]
    force_torque_message.fz = force_torque_values[2]
    force_torque_message.tx = force_torque_values[3]
    force_torque_message.ty = force_torque_values[4]
    force_torque_message.tz = force_torque_values[5]
    return force_torque_message


if __name__ == "__main__":

    client = ForceSensorClient()
    #
    # Put the correct IP address of the computer running the force_sensor_server.py as host!
    #
    client.open_igt_client = pyigtl.OpenIGTLinkClient(host="10.23.0.71", port=18947)

    # Setting up the sensor object
    cal_file="FT28270.cal"
    cal_file_index = 1

    # Vectors for the bias and translation
    bias_readings=[0,0,0,0,0,0]
    translation_floats=[0,0,180,0,0,0]

    # Voltage list
    input_floats=[0,0,0,0,0,0,0]
    output_floats=[0,0,0,0,0,0,0]
    # Units for the sensor values
    translation_dist_unit=atiiaftt.FTUnit.DIST_MM
    translation_angle_unit=atiiaftt.FTUnit.ANGLE_DEG
    force_unit_str=atiiaftt.FTUnit.FORCE_N
    torque_unit_str=atiiaftt.FTUnit.TORQUE_N_M

    print("Using calibration file: '{}'".format(cal_file))
    client.sensor = atiiaftt.FTSensor(cal_file,cal_file_index)

    client.sensor.setToolTransform(translation_floats,translation_dist_unit,translation_angle_unit)
    # Bias the sensor
    bias_readings = client.get_average_voltage_readings()
    print(bias_readings)
    client.sensor.bias(bias_readings)

    # Initialize the publisher and node
    force_torque_publisher = rospy.Publisher("ATI_force_torque", ForceTorque, queue_size=10)
    rospy.init_node("votlage_to_force_converter", anonymous=True)

    # Start conversion and publishing loop
    while True:
        messages = client.open_igt_client.get_latest_messages()
        for message in messages:
            force_torque_values = client.convert_voltage_to_force(message)
            print("Forces: {}".format(force_torque_values))
            force_torque_publisher.publish(create_force_torque_message(force_torque_values))
        if client.reset_flag:
            print("Resetting sensor readings...")
            bias_readings = client.get_average_voltage_readings()
            client.sensor.bias(bias_readings)
            print("Done!")
            client.reset_flag = False
        if rospy.is_shutdown():
            print("Shutting down")
            client.open_igt_client.stop()
            sys.exit()