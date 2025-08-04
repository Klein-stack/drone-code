from dronekit import connect, VehicleMode, LocationGlobalRelative
from time import sleep


print("Connecting to the vehicle . . .")
connection_string = "/dev/ttyACM0"
vehicle = connect(connection_string, wait_ready = True, baud = 57600)

def arm_and_takeoff(target_alt):
    print("Conducting saftey checks . . ." )
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable . . .Conducting safety checks . . .")
        sleep(3)
        
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed or vehicle.mode.name != "GUIDED":
        print("Motors are being armed . . .")
        sleep(3)
    
    vehicle.simple_takeoff(target_alt)
    while True:
        print(f"Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= target_alt*0.95:
            sleep(1)
            break
            


#Parsing kml file function
def parse_kml():
    pass
#
#
#


#Yellow detection using opencv
def detect_yellow():
    pass
#
#
#
#

#Main function
def scan_kml_and_fly():
    pass
#
#
#
#
#
#



            




