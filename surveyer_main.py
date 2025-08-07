from dronekit import connect, VehicleMode, LocationGlobalRelative
from time import sleep
import xml.etree.ElementTree as ET
import os

file = "College_playground_5m_altitude"
print("Connecting to the vehicle . . .")
connection_string = "/dev/ttyACM0"
vehicle = connect(connection_string, wait_ready = True, baud = 57600)

#basic arming
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
    
    
#Parsing kml file function
def parse_kml(file):
    tree = ET.parse(file)
    root = tree.getroot()
    ns = {'kml': 'http://www.opengis.net/kml/2.2'}
    coordinates = []
    for coord in root.findall('.//kml:coordinates', ns):
        for line in coord.text.strip().split():
            parts = line.split(',')
            lon = float(parts[0])
            lat = float(parts[1])
            alt = float(parts[2]) if len(parts) > 2 else None
            coordinates.append((lon, lat, alt) if alt is not None else (lon, lat))
            
    #Save coordinates
    output_path = os.path.join(os.path.dirname(__file__), "survey_coordinates.txt")
    with open(output_path, "w") as f:
    for coord in coords:
        f.write(f"{coord}\n")
    print(f"Coordinates saved to {output_path}")
    return coordinates

#Yellow detection using opencv
def detect_yellow():
    pass
#
#
#
#


#Fly to waypoints
def go_to_waypoints(lat, lon, alt):
#
#
#


#Main function
def scan_kml_and_fly():
    coord = parse_kml(file)
    print(coord)
    
    
    
    
    
    
#
#
#
#
#
#



            




