from dronekit import connect, VehicleMode, LocationGlobalRelative
from time import sleep
import socket
import threading

#connecting to the vehicle
connection_string = "dev/ttyACM0"
vehicle = connect(connection_string, wait_ready=True, baud=57600)

#creating socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rasp_B_ip = "192.2.168.2"
rasp_B_port = 9999

#creating a flag for sending gps
flag = True

#function to send gps
def send_gps():
    global flag
	while flag:
		gps = vehicle.location.global_frame
		if gps.lat != None and gps.lon != None:
			data = f"{gps.lat},{gps.lon},{gps.alt}"
			sock.send(data.encode(),(rasp_B_ip, rasp_B_port))
            
            

		sleep(0.5)
        
#threading
t1 = threading.Thread(target = send_gps)
t1.start()

while True:
    try:
        ask = input("Stop sending gps? Y/N")
        if ask == "Y":
            t1.join()
            flag = False
            print("Stopped sending gps . . .")
            break
            

           
		
	

			
			
		
			
			
	