from dronekit import connect, VehicleMode, LocationGlobalRelative
from time import sleep
import socket
import threading

# Connecting to the vehicle
connection_string = "/dev/ttyACM0"  
vehicle = connect(connection_string, wait_ready=True, baud=57600)

# Creating socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rasp_B_ip = "192.168.2.2"  
rasp_B_port = 9999

# Flag for sending GPS
flag = True

# Function to send GPS
def send_gps():
    global flag
    while flag:
        gps = vehicle.location.global_frame
        if gps.lat is not None and gps.lon is not None:
            data = f"{gps.lat},{gps.lon},{gps.alt}"
            sock.sendto(data.encode(), (rasp_B_ip, rasp_B_port))
        sleep(0.5)

t1 = threading.Thread(target=send_gps)
t1.start()

# Main loop to stop sending GPS
while True:
    try:
        ask = input("Stop sending GPS? Y/N: ").strip().upper()
        if ask == "Y":
            flag = False
            t1.join()
            print("Stopped sending GPS.")
            break
    except KeyboardInterrupt:
        flag = False
        t1.join()
        print("\nStopped sending GPS due to KeyboardInterrupt.")
        break