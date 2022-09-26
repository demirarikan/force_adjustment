import pyigtl

client = pyigtl.OpenIGTLinkClient(host="10.23.0.71", port=18947)

while True:
    message = client.wait_for_message("Text", timeout=3)
    if message is not None:
        voltage_vals_str = message.string
        voltage_vals_list = voltage_vals_str[1:-1].rsplit(" ")
        while "" in voltage_vals_list:
            voltage_vals_list.remove('')
        voltage_vals_list = [float(x) for x in voltage_vals_list]
        print(voltage_vals_list)
