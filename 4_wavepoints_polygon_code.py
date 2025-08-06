from dronekit import connect,VehicleMode,LocationGlobalRelative,Command
import time
from pymavlink import mavutil


print("Connecting to localhhoppst:")
vehicle = connect('127.0.0.1:14551',wait_ready = True)


cmds = vehicle.commands
cmds.clear()
cmds.upload()

print("cleared prev missions")

home = vehicle.location.global_relative_frame
lat = home.lat
lon = home.lon
alt = 4

#cmds.add(Command(0,0,0,
#       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#       mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#       0,1,0,0,0,0,
#       lat,lon,alt))

cmds.add(Command(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0,
    lat, lon, alt))

cmds.add(Command(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0,
    lat +0.0000269 , lon, alt))

cmds.add(Command(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0,
    lat + 0.0000269, lon +0.0000143, alt))

cmds.add(Command(
    0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 0, 0, 0, 0,
    lat, lon + 0.000014, alt))


#cmds.add(Command(
#    0, 0, 0,
#    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
#    0, 1, 0, 0, 0, 0,
#    0, 0, 10))

cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
    0, 1, 5, 0, 0, 0,
    lat, lon, alt))

cmds.add(Command(0, 0, 0,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 1, 0, 0, 0, 0,
    lat, lon, 0))

cmds.upload()
print("Mission uploloaded succesfully")




vehicle.mode=VehicleMode("GUIDED")

print("Checking if vehicle is armable...")
while not vehicle.is_armable:
    print(" Waiting for vehicle to initialise...")
    time.sleep(1)

vehicle.armed = True


while not vehicle.armed:
        print("WAiting for arming")
        time.sleep(1)
print("Drone armed")

print("Taking off manually in GUIDED...")
vehicle.simple_takeoff(4)


while True:
    alt = vehicle.location.global_relative_frame.alt
    print(f" Altitude: {alt:.2f} m")
    if alt >= 3.5:
        print("Target altitude reached")
        break
    time.sleep(1)


vehicle.groundspeed = 1
vehicle.mode = VehicleMode("AUTO")
print("Switched to AUTO. Mission starting...")

next_wp = vehicle.commands.next
while next_wp< vehicle.commands.count:
        if vehicle.commands.next != next_wp:
                next_wp=vehicle.commands.next
                print(f"reached Wasypoint {next_wp}")
        time.sleep(1)


print("Mission done, switching landing...")

#vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt > 1:
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f} m")
    time.sleep(1)


print("Drone has landed. Closing vehicle connection.")
vehicle.close()
