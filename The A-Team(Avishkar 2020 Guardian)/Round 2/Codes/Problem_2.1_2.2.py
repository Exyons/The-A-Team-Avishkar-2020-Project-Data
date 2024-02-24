
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import numpy as np


connection_string_1 = 'udp:127.0.0.1:14550'
connection_string_2 = 'udp:127.1.1.1:14551'

# Connect to the Vehicle
print('Connecting to vehicle 1 on: ', connection_string_1)
vehicle_1 = connect(connection_string_1, wait_ready=True) 
print('Connecting to vehicle 2 on: ', connection_string_2)
vehicle_2 = connect(connection_string_2, wait_ready=True)

CIRCLE_RADIUS  = 5  # Radius of circle
EIGHT_RADIUS   = 7   # Radius of circles in 8
EARTH_RADIUS   = 6378137.0
aTargetAltitude_1 = 45
aTargetAltitude_2 = 15

def get_vertical_eight_points(original_Location):
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
	coords3 = [(x, -2.32*EIGHT_RADIUS + math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(0, -EIGHT_RADIUS, num = 25)] 

    # Fourth, Inverted Semi-circle of lower circle in 8
	coords4 = [(x, -2.32*EIGHT_RADIUS - math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(-EIGHT_RADIUS, EIGHT_RADIUS, num = 25)] 

    # Fifth, Left(Bacha Hua) Upright Quarter-circle of lower circle in 8
	coords5 = [(x, -2.32*EIGHT_RADIUS + math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(EIGHT_RADIUS, 0, num = 30)]

    # Sixth, Left Quarter-circle of the upper circle in 8
	coords6 = [(x, -math.sqrt(EIGHT_RADIUS**2 - x**2 )) for x in np.linspace(0, -EIGHT_RADIUS, num = 15)] 

	lats_longs = []

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
	for (x, y) in coords1:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_1))

	for (x, y) in coords2:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_1))

	for (x, y) in coords3:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_1))

	for (x, y) in coords4:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_1))

	for (x, y) in coords5:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_1))

	for (x, y) in coords6:
		newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
		
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_1))
    
	return lats_longs

def get_vertical_circle_points(original_Location):
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
        
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_2))

	for (x, y) in coords2:
		newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
        
		lats_longs.append(LocationGlobal(b1, newlon, y + aTargetAltitude_2))

	lats_longs.append(original_Location)
	return lats_longs

def get_horizontal_eight_points(original_Location):
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


	coords6 += coords1[:7]

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

def get_horizontal_circle_points(original_Location):
	'''
    This function returns LocationGlobal object 
    It solves a specific equation in cartesian system
    and converts those coodinates into latitude and longitude
    '''
	a1, b1 = original_Location.lon, original_Location.lat # Getting current location coordinates


    # Solving the equation x^2 + y^2 = r^2 to get points and taking origin as centre or current location as centre
    # Multipliction of 1.17 is to make the circle more smooth
	
	coords1 = [(x, np.sqrt(CIRCLE_RADIUS**2 - x**2 )) for x in np.linspace(0, CIRCLE_RADIUS, num = 17)]
    # First, Upright Semi-Circle

    # Second, Inverted Semi-Circle
	coords2 = [(x, -np.sqrt(CIRCLE_RADIUS**2 - x**2 )) for x in np.linspace(CIRCLE_RADIUS, -CIRCLE_RADIUS, num = 15)]

	coords3 = [(x, np.sqrt(CIRCLE_RADIUS**2 - x**2 )) for x in np.linspace(-CIRCLE_RADIUS, 0, num = 15)]
    
	coords3.extend(coords1)

	lats_longs = [original_Location]

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
	
	lats_longs.append(original_Location)

	return lats_longs

def adds_vertical_circle_mission(aLocation, vehicle):
	"""
	Adds a takeoff command and waypoint commands to the current mission. 
	The function assumes vehicle.commands matches the vehicle mission state 
	Returns the last checkpoint number to end the mission 
	"""
	for i in range(2):
		cmds = vehicle.commands
		cmds.clear()
		cmds.upload()
		cmds.wait_ready()

		cmds.download()
		cmds.wait_ready()
		print(" Define/add new commands.")
		# Add new commands. The meaning/order of the parameters is documented in the Command class. 
         
		#Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
		cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, aTargetAltitude_2))
		lats_longs = get_vertical_circle_points(aLocation)

		for LocObj in lats_longs:
			cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, LocObj.lat, LocObj.lon, LocObj.alt))
		print(" Upload new commands to vehicle")
		cmds.upload()
		cmds.wait_ready()

	return len(lats_longs)

		

def adds_vertical_eight_mission(aLocation, vehicle):
    """
    Adds a takeoff command and waypoint commands to the current mission. 
    The function assumes vehicle.commands matches the vehicle mission state 
    Returns the last checkpoint number to end the mission 
    """ 
    for i in range(2):
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()

        print('Clearing exixting commands')
        cmds.clear()
        cmds.upload()
        cmds.wait_ready()

        print(" Define/add new commands.")
        # Add new commands. The meaning/order of the parameters is documented in the Command class.
         
        #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, aTargetAltitude_1))
        lats_longs = get_vertical_eight_points(aLocation)

        for LocObj in lats_longs:
            cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, LocObj.lat, LocObj.lon, LocObj.alt))
        print(" Upload new commands to vehicle")
        cmds.upload()
        
        cmds.wait_ready()

    return len(lats_longs)

		
def adds_horizontal_circle_mission(aLocation2, vehicle):
	"""
	Uses simple_goto function for moving the drone
	"""	
	lats_longs = get_horizontal_circle_points(aLocation2)
	for i in range(len(lats_longs)):
		vehicle.simple_goto(lats_longs[i])

		if i == 0:
			time.sleep(1.5)
		else:
			time.sleep(0.6)

		if i == len(lats_longs) - 1:
			return True


def adds_horizontal_eight_mission(aLocation2, vehicle):
	"""
	Adds a takeoff command and waypoint commands to the current mission. 
	The function assumes vehicle.commands matches the vehicle mission state 
	Returns the last checkpoint number to end the mission 
	"""	
	lats_longs = get_horizontal_eight_points(aLocation2)
	for i in range(len(lats_longs)):
		vehicle.simple_goto(lats_longs[i])

		if i == 0:
			time.sleep(1.5)
		else:
			time.sleep(0.6)

		if i == len(lats_longs) - 1:
			return True


def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
    vehicle.parameters['ARMING_CHECK']=0 # Setting no arming check
    #print("Basic pre-arm checks")

    # Don't let the user try to arm until autopilot is ready
    #while not vehicle.is_armable:
        #print(" Waiting for vehicle to initialise...")
        #time.sleep(1)

        
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

def start_circle():
	'''
	Driving code for circle mission
	'''
	print('Create a new mission...')
	print('\nVehicle 1 at: ', vehicle_1.location.global_frame)
	print('\nVehicle 2 at: ', vehicle_2.location.global_frame)
	
	print('\n\nVehicle 1 Taking off...')
	arm_and_takeoff(aTargetAltitude_2, vehicle_1)
	print('\n\nVehicle 2 Taking off...')
	arm_and_takeoff(aTargetAltitude_2, vehicle_2)
	LAST_WAYPOINT = adds_vertical_circle_mission(vehicle_1.location.global_relative_frame, vehicle_1)
	
	
	vehicle_1.commands.next = 0

	vehicle_1.mode = VehicleMode('AUTO')
	time.sleep(0.2)
	while vehicle_1.mode != 'AUTO':
		print('change Flight mode to "AUTO" in GCS for vehicle 1')
		time.sleep(3)
	time.sleep
	
	FLAG = adds_horizontal_circle_mission(vehicle_2.location.global_frame, vehicle_2)

	print("Now, Ending mission")

	# Reset mission set to first (0) waypoint
	# Set mode to AUTO to start mission
	if vehicle_2.commands.next == LAST_WAYPOINT + 1:
		print('\n\nVehicle 1 completed its mission')
		vehicle_1.mode = VehicleMode("RTL")
	if FLAG:
		print('\n\nVehicle 2 completed its mission')
		vehicle_2.mode = VehicleMode("RTL")

	time.sleep(1)
	while vehicle_1.mode != 'RTL' or vehicle_2.mode != 'RTL':
		print('\n\nChange Flight Mode to "RTL" for both in GCS')
		time.sleep(3)


def start_eight():
	'''
	Driving code for eight mission
	'''
	print('Create a new mission...')
	print('\nVehicle 1 at: ', vehicle_1.location.global_frame)
	print('\nVehicle 2 at: ', vehicle_2.location.global_frame)
	
	print('\n\nVehicle 1 Taking off...')
	arm_and_takeoff(aTargetAltitude_1, vehicle_1)
	print('\n\nVehicle 2 Taking off...')
	arm_and_takeoff(aTargetAltitude_2, vehicle_2)
	LAST_WAYPOINT = adds_vertical_eight_mission(vehicle_1.location.global_relative_frame, vehicle_1)
	
	
	vehicle_1.commands.next = 0

	vehicle_1.mode = VehicleMode('AUTO')
	time.sleep(0.2)
	while vehicle_1.mode != 'AUTO':
		print('change Flight mode to "AUTO" in GCS for vehicle 1')
		time.sleep(3)
	time.sleep
	
	FLAG = adds_horizontal_eight_mission(vehicle_2.location.global_frame, vehicle_2)

	print("Now, Ending mission")

	# Reset mission set to first (0) waypoint
	# Set mode to AUTO to start mission
	if vehicle_2.commands.next == LAST_WAYPOINT + 1:
		time.sleep(8)
		vehicle_1.mode = VehicleMode("LAND")
	if FLAG:
		vehicle_2.mode = VehicleMode("LAND")

	time.sleep(1)


	while vehicle_1.mode != 'LAND' or vehicle_2.mode != 'LAND':
		print('\n\nChange Flight Mode to "LAND" for both in GCS')
		time.sleep(3)



if __name__ == '__main__':
	
	print('\n\nExecuting Eight Mission!!!!:8')
	start_eight()
	while (vehicle_1.armed or vehicle_2.armed):
		print('Wait for vehicles to disarm...')
		time.sleep(2)
	print('\n\nExecuting Circle Mission!!!!:O')
	start_circle()

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle_1.close()
vehicle_2.close()