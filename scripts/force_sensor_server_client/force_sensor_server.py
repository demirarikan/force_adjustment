import nidaqmx
import xml.etree.ElementTree as ET
import numpy as np
import pyigtl
from time import sleep


# Reads the voltage data from the sensor and returns it as a numpy array once.
def read_voltage_data():
    with nidaqmx.Task() as task:
        task.ai_channels.add_ai_voltage_chan("Dev1/ai0:5")
        voltage_values = task.read()
    return np.asarray(voltage_values)


def main():
    # start openigtlink server
    print('STARTING SERVER...')
    server = pyigtl.OpenIGTLinkServer(port=18947, local_server=False)

    while True:
        # Wait for client to connect
        if not server.is_connected():
            print('Waiting for client to connect...', end="\r")
            sleep(0.1)
            continue

        # Create string message with voltage data
        string_message = pyigtl.StringMessage(
            str(read_voltage_data()), device_name="voltage_server"
        )

        server.send_message(string_message)


if __name__ == "__main__":
    main()
