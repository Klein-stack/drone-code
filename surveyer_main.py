from dronekit import connect, VehicleMode, LocationGlobalRelative
from time import sleep


print("Connecting to the vehicle . . .")
connection_string = "/dev/tty/ACM0"
vehicle = connect(connection_string, wait_ready = True, baud = 56700)

def arm_and_takeoff(target_alt):
    print("Conducting saftey checks . . ." )
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable . . .Conducting safety checks . . .")
        sleep(5)
        
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed and not vehicle.mode == GUIDED:
        print("Motors are being armed . . .")
        sleep(5)
    
    vehicle.simple_takeoff(target_alt)
    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame >= target_alt.0.95:
            break
            sleep(5)


#Parsing kml file function
def parse_kml()
#
#
#
#


#Yellow detection using opencv
def detect_yellow()
#
#
#
#


            




