from time import sleep


for i in range(100):
    print(i, end='\r')
    sleep(0.01)
    print("", end="\r")