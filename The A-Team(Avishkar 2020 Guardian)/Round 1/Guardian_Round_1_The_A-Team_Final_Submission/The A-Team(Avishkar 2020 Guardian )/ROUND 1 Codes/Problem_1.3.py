
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import numpy as np


#Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


# CONSTANTS
CIRCLE_RADIUS    = 2.5  # Radius of verticle circle
EARTH_RADIUS     = 6378137.0
EIGHT_RADIUS     = 10   # Radius of circular part of verticle 8
aTargetAltitude1 = 15   # Altitude for executing verticle circle
aTargetAltitude2 = 45   # Altitude for executing verticle 8

def get_vertcircle_points(original_Location,INITIAL_ALT):
    '''
    This function returns LocationGlobal object 
    It solves a specific equation in cartesian system
    and converts those coodinates into latitude and longitude
    '''
    a1, b1 = original_Location.lon, original_Location.lat # Getting current location coordinates


    # Solving the equation x^2 + y^2 = r^2 to get points and taking origin as centre or current location as centre
    # Multipliction of 1.17 is to make the circle more smooth
    coords1 = [(x, np.sqrt(CIRCLE_RADIUS**2 - x**2 )*1.17) for x in np.linspace(-CIRCLE_RADIUS, CIRCLE_RADIUS, num = 15)]
    # First, Upright Semi-Circle

    # Second, Inverted Semi-Circle
    coords2 = [(x, -np.sqrt(CIRCLE_RADIUS**2 - x**2 )) for x in np.linspace(CIRCLE_RADIUS, -CIRCLE_RADIUS, num = 18)] 
    
    lats_longs = []

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
    for (x, y) in coords1:
        newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
        
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))

    for (x, y) in coords2:
        newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
        
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))
    return lats_longs

def get_virteight_points(original_Location, INITIAL_ALT):
    '''
    This function returns LocationGlobal object 
    It solves a specific equation in cartesian system
    and converts those coodinates into latitude and longitude
    '''
    a1, b1 = original_Location.lon, original_Location.lat

    # Solving the equation x^2 + y^2 = r^2 to get points and taking origin as centre or current location as centre

    # First, Upright Semi-Circle of upper circle in 8
    coords1 = [(x, math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(-EIGHT_RADIUS, EIGHT_RADIUS, num = 30)]

    # Second, Inverted Quarter-Circle of upper circle in 8
    coords2 = [(x, -math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(EIGHT_RADIUS, 0, num = 15)]

    # Third, Upright Quarter-circle of lower circle in 8
    coords3 = [(x, -2.32*EIGHT_RADIUS*math.sin(60*math.pi/180) + math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(0, -EIGHT_RADIUS, num = 25)] 

    # Fourth, Inverted Semi-circle of lower circle in 8
    coords4 = [(x, -2.32*EIGHT_RADIUS*math.sin(60*math.pi/180) - math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(-EIGHT_RADIUS, EIGHT_RADIUS, num = 25)] 

    # Fifth, Left(Bacha Hua) Upright Quarter-circle of lower circle in 8
    coords5 = [(x, -2.32*EIGHT_RADIUS*math.sin(60*math.pi/180) + math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(EIGHT_RADIUS, 0, num = 30)]

    # Sixth, Left Quarter-circle of the upper circle in 8
    coords6 = [(x, -math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(0, -EIGHT_RADIUS, num = 15)] 

    lats_longs = []

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
    for (x, y) in coords1:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))

    for (x, y) in coords2:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))

    for (x, y) in coords3:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))

    for (x, y) in coords4:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))

    for (x, y) in coords5:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))

    for (x, y) in coords6:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(b1, newlon, y + INITIAL_ALT))
    
    
    return lats_longs


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5



def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint


def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

    return len(lats_longs)

def adds_verteight_mission(aLocation, aTargetAltitude):
    """
    Adds a takeoff command and waypoint commands to the current mission. 
    The function assumes vehicle.commands matches the vehicle mission state 
    Returns the last checkpoint number to end the mission 
    """ 
    for i in range(3):
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()

        print(" Clear any existing commands")
        cmds.clear()
        cmds.upload()
        cmds.wait_ready()

        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class.
         
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 45))
        lats_longs = get_virteight_points(aLocation, aTargetAltitude)

        for LocObj in lats_longs:
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, LocObj.lat, LocObj.lon, LocObj.alt))
        print(" Upload new commands to vehicle")
        cmds.upload()
        
        cmds.wait_ready()

    return len(lats_longs)

def adds_vertcircle_mission(aLocation, aTargetAltitude):
    """
    Adds a takeoff command and waypoint commands to the current mission. 
    The function assumes vehicle.commands matches the vehicle mission state 
    Returns the last checkpoint number to end the mission 
    """ 
    for i in range(3): 
        cmds = vehicle.commands
        cmds.clear()
        cmds.upload()
        cmds.wait_ready()

        cmds.download()
        cmds.wait_ready()

        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class. 
         
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 15))
        lats_longs = get_vertcircle_points(aLocation, aTargetAltitude)

        for LocObj in lats_longs:
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, LocObj.lat, LocObj.lon, LocObj.alt))
        print(" Upload new commands to vehicle")
        cmds.upload()
        cmds.upload()

        cmds.wait_ready()

    return len(lats_longs)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")

    # Copter should arm in GUIDED mode
    # As when this script is unable to change the flight mode
    # asking user to do it manually
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode == 'GUIDED':
        print('Change Flight mode manually to "GUIDED" in GCS')
        time.sleep(3)

    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto 
    #(otherwise the command after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def start_vertcircle():
    '''
    Driving code for vertcal circle mission
    '''
    print('Create a new mission (for current location)')
    print('\n\n',vehicle.location.global_frame)
    LAST_WAYPOINT = adds_vertcircle_mission(vehicle.location.global_frame, aTargetAltitude1)

    
    vehicle.mode = VehicleMode('STABILIZE')
    vehicle.armed = True

    arm_and_takeoff(aTargetAltitude1)
    
    print("Now, Starting mission")

    # Reset mission set to first (0) waypoint
    vehicle.commands.next = 0

    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")
    while not vehicle.mode == 'AUTO':
            print('Change Flight mode manually to "AUTO" in GCS')
            time.sleep(3)

    while True:
        nextwaypoint=vehicle.commands.next
        
        if nextwaypoint == LAST_WAYPOINT+1 and distance_to_current_waypoint() < 0.15:
            print('Mission Complete!!!')
            vehicle.mode = VehicleMode("RTL")
            time.sleep(0.5)
            while not vehicle.mode == 'RTL':
                print('Change Flight mode manually to "RTL" in GCS')
                time.sleep(3)
            
            break
        else:
            print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))

        time.sleep(1)

def start_verteight():
    '''
    Driving code for vertical eight mission
    '''
    print('Create a new mission (for current location)')
    print('\n\n',vehicle.location.global_frame)
    LAST_WAYPOINT = adds_verteight_mission(vehicle.location.global_frame, aTargetAltitude2)

    # When using airsim uncomment the below two line
    vehicle.mode = VehicleMode('STABILIZE')
    vehicle.armed = True

    arm_and_takeoff(aTargetAltitude2)
    print("Now, Starting mission")

    # Reset mission set to first (0) waypoint
    vehicle.commands.next = 0

    # Set mode to AUTO to start mission
    vehicle.mode = VehicleMode("AUTO")
    while not vehicle.mode == 'AUTO':
        print('Change Flight mode manually to "AUTO" in GCS')
        time.sleep(3)

    while True:
        nextwaypoint=vehicle.commands.next
        

        if nextwaypoint == LAST_WAYPOINT+1 and distance_to_current_waypoint() < 0.5:
            print('Mission Complete!!! ')
            break
        else:
            print('Distance to waypoint ({}): {}'.format(nextwaypoint, distance_to_current_waypoint()))
        time.sleep(1)

def start_flip():
    '''
    Driving code for flip
    '''
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise... ")
        time.sleep(1)

    while vehicle.mode != 'STABILIZE':
        vehicle.mode = VehicleMode('STABILIZE')
        print("Change Flight mode manually to 'STABILIZE' in GCS")
        time.sleep(1)

    while not vehicle.armed:
        print("Arming motors")
        vehicle.armed = True
        time.sleep(0.5)

    #Doing Channel Override
    count = 0
    while True:
        vehicle.channels.overrides[3] = 1800 # Override throttle
        print (" Ch3 override: ", vehicle.channels.overrides[3])

        if vehicle.location.global_relative_frame.alt > 30:
            
            vehicle.mode = VehicleMode('FLIP') #Changing flight mode to flip
            count += 1
            

        print('Height reached {}'.format(vehicle.location.global_relative_frame.alt))
        if count > 5:
            break
        time.sleep(1)


    while vehicle.mode != 'RTL':
        vehicle.mode = VehicleMode('RTL')
        print("Change Flight mode manually to 'RTL' in GCS")
        time.sleep(1)
        break
    time.sleep(0.5)

if __name__ == '__main__':
    print('\n\nExecuting mission verticle circle O!!!!:O')
    start_vertcircle()

    while vehicle.armed:
        print('Wait till drone disarms...')
        time.sleep(2)

    print('\n\nExecuting mission verticle 8!!!:8')
    start_verteight()

    print('\n\nStarting Flip!!!!')
    start_flip()

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
