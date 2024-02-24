
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


PRIMARY_RADIUS = 5  # Radius of starting circle in spiral
EIGHT_RADIUS   = 10 # Radius of circles in 8
EARTH_RADIUS   = 6378137.0
SIDE_LENGTH    = 35 # Hexagon Side Length

def get_hexagon_points(original_Location):
	'''
	This function returns LocationGlobal object 
	It converts cartesian coodinates into latitude and longitude
	'''

	a1, b1 = original_Location.lon, original_Location.lat # Original Location

    # Coordinates of vertices of hexagon in cartesian coordinates
    # Length of side is SIDE_LENGTH
	coords =  [(SIDE_LENGTH, 0), # First Coordinate(Used as starting point)
    
    (SIDE_LENGTH + SIDE_LENGTH*math.cos(30*math.pi/180), SIDE_LENGTH*math.sin(30*math.pi/180)), # Second Coordinate
    
    (SIDE_LENGTH, SIDE_LENGTH*math.sin(30*math.pi/180)*2 ), # Third Coordinate
    
    (0, SIDE_LENGTH), # Fourth Coordinate
    
    (-SIDE_LENGTH*math.cos(30*math.pi/180), SIDE_LENGTH*math.sin(30*math.pi/180)), # Fifth Coordinate

	(0,0)] # Sixth Coordinate, back to start

	lats_longs = [] # Empty list for storing latitude ans longitude

    # Taking home location as starting point
    # Finding other coordinates using cartesian to latitude and longitude transformation
	for (x, y) in coords:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(b1*math.pi/180))
		newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))
    
	return lats_longs

def get_spiral_points(original_Location):
	'''
	This function returns LocationGlobal object 
	It solves a specific equation in cartesian system
	and converts those coodinates into latitude and longitude
	'''
	a1, b1 = original_Location.lon, original_Location.lat # Getting current location coordinates


    # Solving the equation x^2 + y^2 = r^2 to get points and taking origin as centre or current location as centre
	coords1 = [(x, np.sqrt(PRIMARY_RADIUS**2 - x**2 )) for x in np.linspace(-PRIMARY_RADIUS, PRIMARY_RADIUS, num = 40)] # First, Upright Semi-Circle

    # Second, Inverted Semi-Circle
	coords2 = [(x - PRIMARY_RADIUS, -np.sqrt((PRIMARY_RADIUS*2)**2 - x**2 )) for x in np.linspace(PRIMARY_RADIUS*2, -PRIMARY_RADIUS*2, num = 30)]

    # Third, Upright Semi-Circle
	coords3 = [(x, np.sqrt((PRIMARY_RADIUS*3)**2 - x**2 )) for x in np.linspace(-PRIMARY_RADIUS*3, PRIMARY_RADIUS*3, num = 40)] 

    # Fourth, Inverted Semi-Circle
	coords4 = [(x - PRIMARY_RADIUS, - np.sqrt((PRIMARY_RADIUS*4)**2 - x**2 )) for x in np.linspace(PRIMARY_RADIUS*4, -PRIMARY_RADIUS*4, num = 30)]

    # Fifth, Upright Semi-Circle
	coords5 = [(x, np.sqrt((PRIMARY_RADIUS*5)**2 - x**2 )) for x in np.linspace(-PRIMARY_RADIUS*5, PRIMARY_RADIUS*5, num = 40)] 

    # Sixth, Inverted Semi-Circle
	coords6 = [(x - PRIMARY_RADIUS, - np.sqrt((PRIMARY_RADIUS*6)**2 - x**2 )) for x in np.linspace(PRIMARY_RADIUS*6, -PRIMARY_RADIUS*6, num = 30)] 

	lats_longs = [original_Location] # Starting mission from Home Location

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
	for (x, y) in coords1:
		newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
		newlat = b1 + y*180/(np.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords2:
		newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
		newlat = b1 + y*180/(np.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords3:
		newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
		newlat = b1 + y*180/(np.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords4:
		newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
		newlat = b1 + y*180/(np.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords5:
		newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
		newlat = b1 + y*180/(np.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords6:
		newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
		newlat = b1 + y*180/(np.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))
    
	return lats_longs

def get_eight_points(original_Location):
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
	coords3 = [(x, -2.32*EIGHT_RADIUS*math.sin(60*math.pi/180) + math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(0, -EIGHT_RADIUS, num = 15)] 

    # Fourth, Inverted Semi-circle of lower circle in 8
	coords4 = [(x, -2.32*EIGHT_RADIUS*math.sin(60*math.pi/180) - math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(-EIGHT_RADIUS, EIGHT_RADIUS, num = 30)] 

 	# Fifth, Left(Bacha Hua) Upright Quarter-circle of lower circle in 8
	coords5 = [(x, -2.32*EIGHT_RADIUS*math.sin(60*math.pi/180) + math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(EIGHT_RADIUS, 0, num = 15)]

    # Sixth, Left Quarter-circle of the upper circle in 8
	coords6 = [(x, -math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(0, -EIGHT_RADIUS, num = 15)] 

	lats_longs = [original_Location] # Starting mission from Home Location

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
	for (x, y) in coords1:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords2:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords3:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords4:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords5:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

	for (x, y) in coords6:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
		lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))
	
	lats_longs.append(original_Location) # Ending at starting location
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

def adds_hexagon_mission(aLocation):
    """
    Adds a takeoff command and waypoint commands to the current mission. 
    The function assumes vehicle.commands matches the vehicle mission state 
    Returns the last checkpoint number to end the mission 
    """	
    for i in range(2):
	    cmds = vehicle.commands
	    cmds.download()
	    cmds.wait_ready() # wait until download is complete.

	    print(" Clear any existing commands")
	    cmds.clear() 
	    
	    print(" Define/add new commands.")
	    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
	     
	    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
	    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 15))
	    lats_longs = get_hexagon_points(aLocation)

	    # Add waypoint commands for mission
	    for LocObj in lats_longs:
	    	cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, LocObj.lat, LocObj.lon, 15))
	    print(" Upload new commands to vehicle")
	    cmds.upload()
	    cmds.wait_ready() # wait until upload is complete.

    return len(lats_longs)

def adds_spiral_mission(aLocation):
	"""
	Adds a takeoff command and waypoint commands to the current mission. 
	The function assumes vehicle.commands matches the vehicle mission state 
	Returns the last checkpoint number to end the mission 
	"""
	for i in range(2):
		cmds = vehicle.commands
		cmds.download()
		cmds.wait_ready()

		cmds.clear()
		cmds.upload()
		cmds.wait_ready()

		print(" Define/add new commands.")
		# Add new commands. The meaning/order of the parameters is documented in the Command class. 
		 
		#Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
		cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 15))
		lats_longs = get_spiral_points(aLocation)

		# Add waypoint commands for mission
		for LocObj in lats_longs:
		    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, LocObj.lat, LocObj.lon, 15))
		print(" Upload new commands to vehicle")
		cmds.upload()
		cmds.upload()

		cmds.wait_ready()

	return len(lats_longs)

def adds_eight_mission(aLocation):
	"""
	Adds a takeoff command and waypoint commands to the current mission. 
	The function assumes vehicle.commands matches the vehicle mission state 
	Returns the last checkpoint number to end the mission 
	"""	
	for i in range(2):
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
		cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 15))
		lats_longs = get_eight_points(aLocation)

		# Add waypoint commands for mission
		for LocObj in lats_longs:
		    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, LocObj.lat, LocObj.lon, 15))
		print(" Upload new commands to vehicle")
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

    vehicle.armed = True # Setting vehicle Armed

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

def start_spiral():
	'''
	Driving code for spiral mission
	'''
	print('Create a new mission (for current location)')
	print('\n\n',vehicle.location.global_frame)
	LAST_WAYPOINT = adds_spiral_mission(vehicle.location.global_frame)

	arm_and_takeoff(15)

	vehicle.airspeed = 1.5 # Sets vehicle speed

	print("Now, Starting mission")

	# Reset mission set to first (0) waypoint
	vehicle.commands.next = 0

	# Set mode to AUTO to start mission
	vehicle.mode = VehicleMode("AUTO")
	time.sleep(1)
	while not vehicle.mode == 'AUTO':
	    	print('Change Flight mode manually to "AUTO" in GCS')
	    	time.sleep(3)

	while True:
		nextwaypoint=vehicle.commands.next
		print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
		if nextwaypoint == LAST_WAYPOINT+1 and distance_to_current_waypoint() < 1:
			print('Spiral Mission Complete!!!')
			break
		time.sleep(1)
	vehicle.mode = VehicleMode("RTL")
	time.sleep(1)
	while not vehicle.mode == 'RTL':
	    	print('Change Flight mode manually to "RTL" in GCS')
	    	time.sleep(3)


def start_hexagon():
	'''
	Driving code for hexagon mission
	'''
	print('Create a new mission (for current location)')
	print('\n\n',vehicle.location.global_frame)
	LAST_WAYPOINT = adds_hexagon_mission(vehicle.location.global_frame)


	# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).
	arm_and_takeoff(15)

	print("Now, Starting mission")
	# Reset mission set to first (0) waypoint
	vehicle.commands.next=0

	# Set mode to AUTO to start mission
	vehicle.mode = VehicleMode("AUTO")
	while not vehicle.mode == 'AUTO':
	    	print('Change Flight mode manually to "AUTO" in GCS')
	    	time.sleep(3)

	while True:
	    nextwaypoint=vehicle.commands.next
	    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))

	    time.sleep(1)

	    if distance_to_current_waypoint() <= 9:
	    	vehicle.airspeed = 1.5
	    else:
	    	vehicle.airspeed = 2.3

	    if nextwaypoint == LAST_WAYPOINT + 1 and distance_to_current_waypoint() < 1:
	    	print('Hexagon Mission Complete!!!!')
	    	break

def start_eight():
	'''
	Driving code for 8 mission
	'''
	print('Create a new mission (for current location)')
	print('\n\n',vehicle.location.global_frame)
	
	LAST_WAYPOINT = adds_eight_mission(vehicle.location.global_frame)

	arm_and_takeoff(15)

	vehicle.airspeed = 1.5

	print("Now, Starting mission")
	# Reset mission set to first (0) waypoint
	vehicle.commands.next = 0

	# Set mode to AUTO to start mission
	vehicle.mode = VehicleMode("AUTO")
	time.sleep(1)
	while not vehicle.mode == 'AUTO':
	    	print('Change Flight mode manually to "AUTO" in GCS')
	    	time.sleep(3)

	while True:
		nextwaypoint=vehicle.commands.next
		print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
		if nextwaypoint == LAST_WAYPOINT+1 and distance_to_current_waypoint() < 1:
			print('8 Mission Complete!!!')
			break
		time.sleep(1)

if __name__ == '__main__':
	
	print('\n\nExecuting Heaxagon Mission!!!!')
	start_hexagon()

	print('\n\nExecuting 8 Mission!!!!:8')
	start_eight()

	print('\n\nExecuting Spiral Mission!!!!:@')
	start_spiral()

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
