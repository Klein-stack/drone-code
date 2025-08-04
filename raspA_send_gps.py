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
flag = "Green"

#function to send gps
def send_gps():
	while flag == "Green":
		gps = vehicle.location.global_frame
		if gps.lat != None or gps.lon != None:
			data = f"{gps.lat},{gps.lon},{gps.alt}"
			sock.send(data.encode(),(rasp_A_ip, rasp_B_port))
			ask = input("do you want to continue? Y/N")
			if ask == 'N':
				break
		sleep(0.5)
		
	

			
			
		
			
			
	