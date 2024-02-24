
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import numpy as np


connection_string_1 = 'udp:127.0.0.1:14550'
connection_string_2 = 'udp:127.1.1.1:14551'
connection_string_3 = 'udp:127.2.2.1:14552'

# Connect to the Vehicle
print('Connecting to vehicle 1 on: ', connection_string_1)
vehicle_1 = connect(connection_string_1, wait_ready=True)  # Leader
print('Connecting to vehicle 2 on: ', connection_string_2)
vehicle_2 = connect(connection_string_2, wait_ready=True)  # Follower
print('Connecting to vehicle 3 on: ', connection_string_3)
vehicle_3 = connect(connection_string_3, wait_ready=True)  # Follower

EARTH_RADIUS = 6378137.0
SIDE_LENGTH  = 30

def get_triangle_points(original_Location):
    '''
    This function returns LocationGlobal object 
    It solves a specific equation in cartesian system
    and converts those coodinates into latitude and longitude
    '''
    a1, b1 = original_Location.lon, original_Location.lat

    coords = [(0,0), (SIDE_LENGTH,0), (SIDE_LENGTH/2, SIDE_LENGTH*math.sin(60*math.pi/180))]

    lats_longs = []

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
    for (x, y) in coords:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobalRelative(newlat, newlon, original_Location.alt))
    return lats_longs

def get_line_points(original_Location):
    
    a1, b1 = original_Location.lon, original_Location.lat # Getting current location coordinates

    coords = [(SIDE_LENGTH/2, 0), (SIDE_LENGTH/2, (SIDE_LENGTH*math.sin(60*math.pi/180))/2)]
    
    lats_longs = []

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
    for (x, y) in coords:
        newlon = a1 + x*180/(np.pi*EARTH_RADIUS*np.cos(np.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))

    return lats_longs

def get_L_points(original_Location):
   
    a1, b1 = original_Location.lon, original_Location.lat

    coords = [(SIDE_LENGTH, (SIDE_LENGTH*math.sin(60*math.pi/180))/2)]

    lats_longs = [] # Starting mission from Home Location

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
    for (x, y) in coords:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(newlat, newlon, original_Location.alt))
    return lats_longs

def get_vertical_line_points(original_Location):
   
    a1, b1 = original_Location.lon, original_Location.lat

    coords = [(0, 0, 0), (0, 0, 15), (0, 0, 15*2)]

    lats_longs = [] # Starting mission from Home Location

    # Transforming cartesian coordinates (x, y) into (latitude, longitude)
    for (x, y, z) in coords:
        newlon = a1 + x*180/(math.pi*EARTH_RADIUS*math.cos(math.pi*b1/180))
        newlat = b1 + y*180/(math.pi*EARTH_RADIUS)
        lats_longs.append(LocationGlobal(newlat, newlon, z + original_Location.alt))
    return lats_longs


def adds_triangle_mission(*args):

    lats_longs = get_triangle_points(args[0])
    for i in range(1, len(args)):
        args[i].simple_goto(lats_longs[i-1])
        time.sleep(0.6)
        if i == 3:
            break
    time.sleep(29)
        

def adds_line_mission(*args):

    lats_longs = get_line_points(args[0])
    for i in range(1, len(args)):
        args[i].simple_goto(lats_longs[i-1])
        time.sleep(0.6)
        if i == 3:
            break
    time.sleep(20)

        
def adds_L_mission(*args):

    lats_longs = get_L_points(args[0])
    for i in range(1, len(args)):
        args[i].simple_goto(lats_longs[i-1])
        time.sleep(0.6)
        if i == 3:
            break
    time.sleep(20)


def adds_vertical_line_mission(*args):

    lats_longs = get_vertical_line_points(args[0])
    for i in range(1, len(args)):
        args[i].simple_goto(lats_longs[i-1])
        time.sleep(0.6)
        if i == 3:
            break
    time.sleep(20)


def arm_and_takeoff(aTargetAltitude, vehicle):
    """
    Arms vehicle and fly to aTargetAltitude.
    """
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

def start_triangle():
    '''
    Driving code for triangle mission
    '''
    print('\nVehicle 1 at: ', vehicle_1.location.global_frame)
    print('\nVehicle 2 at: ', vehicle_2.location.global_frame)
    print('\nVehicle 3 at: ', vehicle_3.location.global_frame)

    print('\nVehicle 1 taking off....')
    arm_and_takeoff(15, vehicle_1)

    print('\nVehicle 2 taking off....')
    arm_and_takeoff(15, vehicle_2)

    print('\nVehicle 3 taking off....')
    arm_and_takeoff(15, vehicle_3)
    
    adds_triangle_mission(vehicle_1.location.global_frame, vehicle_1, vehicle_2, vehicle_3)

    print("Now, Ending mission")


def start_line():
    '''
    Driving code for line mission
    '''
    print('\nVehicle 1 at: ', vehicle_1.location.global_frame)
    print('\nVehicle 2 at: ', vehicle_2.location.global_frame)
    print('\nVehicle 3 at: ', vehicle_3.location.global_frame)
    
    adds_line_mission(vehicle_1.location.global_frame, vehicle_1, vehicle_2)

    print("Now, Ending mission")

def start_L():
    '''
    Driving code for L mission
    '''
    print('\nVehicle 1 at: ', vehicle_1.location.global_frame)
    print('\nVehicle 2 at: ', vehicle_2.location.global_frame)
    print('\nVehicle 3 at: ', vehicle_3.location.global_frame)
    
    adds_L_mission(vehicle_1.location.global_frame, vehicle_1)

    print("Now, Ending mission")

    print('All drones now returning to home position')
    vehicle_1.mode = VehicleMode('LAND')
    vehicle_2.mode = VehicleMode('LAND')
    vehicle_3.mode = VehicleMode('LAND')

    while vehicle_1.mode != 'LAND' or vehicle_2.mode != 'LAND' or vehicle_3.mode != 'LAND':
        print('Set mode to "LAND" for every vehicle in GCS')
        time.sleep(3)

def start_vertical_line():
    '''
    Driving code for vertical line mission
    '''
    print('\nVehicle 1 at: ', vehicle_1.location.global_frame)
    print('\nVehicle 2 at: ', vehicle_2.location.global_frame)
    print('\nVehicle 3 at: ', vehicle_3.location.global_frame)

    print('\nVehicle 1 taking off....')
    arm_and_takeoff(15, vehicle_1)

    print('\nVehicle 2 taking off....')
    arm_and_takeoff(15, vehicle_2)

    print('\nVehicle 3 taking off....')
    arm_and_takeoff(15, vehicle_3)
    
    adds_vertical_line_mission(vehicle_1.location.global_frame, vehicle_1, vehicle_2, vehicle_3)

    print("Now, Ending mission")
    vehicle_1.mode = VehicleMode('RTL')
    vehicle_2.mode = VehicleMode('RTL')
    vehicle_3.mode = VehicleMode('RTL')

    while vehicle_1.mode != 'RTL' or vehicle_2.mode != 'RTL' or vehicle_3.mode != 'RTL':
        print('Set mode to "LAND" for every vehicle in GCS')
        time.sleep(3)


if __name__ == '__main__':
    
    print('\n\nExecuting triangle mission!!!!')
    start_triangle()

    print('\n\nExecuting line mission!!!')
    start_line()

    print('\n\nExecuting L mission!!!')
    start_L()

    while (vehicle_1.armed or vehicle_2.armed or vehicle_3.armed):
        print('Wait for vehicles to disarm...')
        time.sleep(3)
    
    print('\n\nExecuting verticle line Mission!!!!')
    start_vertical_line()

#Close vehicle object before exiting script
print("Close vehicle object")
vehicle_1.close()
vehicle_2.close()
vehicle_3.close()