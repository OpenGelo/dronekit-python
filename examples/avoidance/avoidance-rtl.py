from __future__ import print_function

"""
avoidance.py: Demonstrate autopilot collision avoidance functionality

Full documentation is provided at http://python.dronekit.io/examples/avoidance.html
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import threading

import dronekit
import mavlink_hub

#Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Demonstrates autopilot automatic avoidance functionality.')
parser.add_argument('--extra-connection',
                   help="extra MAVLink connections to make.")
parser.add_argument('--binary',
                    help="path to autopilot binary to use")
parser.add_argument('--defaults',
                    help="path to autopilot defaults file to use")
args = parser.parse_args()

# example of a binary path: $HOME/rc/ardupilot/build/sitl-debug/bin/arducopter-quad
# example of a defaults path: $HOME/rc/ardupilot/Tools/autotest/copter_params.parm


sitls = []

hub_thread = None
target_systems = []

hub_connection_strings = []

# start many dronekit-sitl processes
simulation_count = 2
for i in range(0,simulation_count):
    print("Creating simulator (SITL) %d" % (i,))
    from dronekit_sitl import SITL
    sitl = SITL(instance=i, path=args.binary, defaults_filepath=args.defaults)
#    sitl.download('copter', '3.3', verbose=True)
    lat = -35.363261 + i*0.00128
    sitl_args = ['--model', 'quad', '--home='+str(lat)+',149.165230,584,353']
    sitls.append([sitl, sitl_args])
    hub_connection_strings.append(sitl.connection_string())

# start the SITLs one at a time, giving each a unique SYSID_THISMAV
def change_sysid_target(i, new_sysid, connection_string):
    print("%d: Launching SITL (%s)" % (i,str(sitls[i][1])))
    sitls[i][0].launch(sitls[i][1], await_ready=True, verbose=True)
    print("%d: Connecting to its vehicle 1" % (i,))
    vehicle = dronekit.connect(connection_string, wait_ready=True, target_system=1)
    while vehicle.parameters["SYSID_THISMAV"] != new_sysid:
        print("%d: Resetting its SYID_THISMAV to %d" % (i, new_sysid,))
        vehicle.parameters["SYSID_THISMAV"] = new_sysid
        time.sleep(0.1)
    # enable ADSB:
    vehicle.parameters["ADSB_ENABLE"] = 1
    # enable avoidance:
    vehicle.parameters["AVOID_ENABLE"] = 1
    # set avoidance behaviour to RTLK:
    vehicle.parameters["AVOID_BEHAVIOR"] = mavutil.mavlink.MAV_COLLISION_ACTION_RTL
    # set the fail radius down to let us see everything on a reasonable scale:
    vehicle.parameters["AVOID_DIST_F_XY"] = 10
    # set the time horizon down to let us see everything on a reasonable scale:
    vehicle.parameters["AVOID_TIME_F_XY"] = 10

    print("%d: Allowing time for parameter write" % (i,))
    time.sleep(2)
    print("%d: Stop" % (i,))
    sitls[i][0].stop()
    vehicle.disconnect()

change_sysid_threads = []
for i in range(0,len(sitls)):
    new_sysid = len(sitls)-i+1
    change_sysid_threads.append(threading.Thread(target=change_sysid_target,
                                                 args=(i,new_sysid, hub_connection_strings[i])))
    change_sysid_threads[-1].start()
    target_systems.append(new_sysid)
#        mav.remove_message_listener(vehicle)
for thread in change_sysid_threads:
    print("Waiting for thread...")
    thread.join()

print("Sleeping a little to let SITLs go away...")
time.sleep(2)
print("Launching SITLs")
for i in range(0,len(sitls)):
    sitl = sitls[i][0]
    sitl_args = sitls[i][1]
    sitl.launch(sitl_args, await_ready=True, restart=True, use_saved_data=True, wd=sitl.wd, verbose=True)

# create another connection so a GCS can be connected for visualisation
if args.extra_connection:
    hub_connection_strings.append(args.extra_connection)

# dronekit-python (us!) port:
connection_string = "udpout:localhost:2345"
hub_connection_strings.append("udpin:localhost:2345")

hub_should_quit = False
def mavlink_hub_target():
    print("Hub thread starting")
    hub = mavlink_hub.MAVLinkHub(hub_connection_strings)
    hub.init()
    while not hub_should_quit:
        hub.loop()
    print("Hub quitting")
    hub.connection_maintenance_target_should_live = False

print("Starting mavlink_hub thread")
hub_thread = threading.Thread(target=mavlink_hub_target)
hub_thread.start()

# Create a MAVLink connection:
print("Attempting connect to (%s)" % (connection_string,))
mav = dronekit.mavlink.MAVConnection(connection_string)
print("Connected")

vehicles = [None] * len(target_systems)
# Connect to the Vehicle
def vehicle_connect_target(mav, id, offset):
    vehicle = dronekit.connect(mav, wait_ready=True, target_system=id)
    vehicles[offset] = vehicle

offset = 0
connect_threads = []
for id in target_systems:
    print('Connecting to vehicle with system ID (%d)' % id)
    connect_threads.append(threading.Thread(target=vehicle_connect_target,
                                                     args=(mav,id,offset)))
    connect_threads[-1].start()
    offset += 1
    time.sleep(1) # mav is not thread safe?!

for thread in connect_threads:
    print("Waiting for connect thread...")
    thread.join()

def dump_vehicle_state(vehicle):
    print(" Autopilot Firmware version: %s" % vehicle.version)
    print("   Major version number: %s" % vehicle.version.major)
    print("   Minor version number: %s" % vehicle.version.minor)
    print("   Patch version number: %s" % vehicle.version.patch)
    print("   Release type: %s" % vehicle.version.release_type())
    print("   Release version: %s" % vehicle.version.release_version())
    print("   Stable release?: %s" % vehicle.version.is_stable())


for vehicle in vehicles:
    dump_vehicle_state(vehicle)

mav_lock = threading.Lock() # FIXME; move locking into MAVSystem

def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # Copter should arm in GUIDED mode
    while vehicle.mode != 'GUIDED':
        print("Setting mode GUIDED")
        mav_lock.acquire()
        vehicle.mode = VehicleMode("GUIDED")
        mav_lock.release()
        time.sleep(0.5)

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print("Arming motors")
        mav_lock.acquire()
        vehicle.armed = True
        mav_lock.release()
        time.sleep(0.5)

    print("Taking off!")
    mav_lock.acquire()
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
    mav_lock.release()

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Mode: %s  Altitude: ", vehicle.mode, vehicle.location.global_relative_frame.alt)
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def vehicle_launcher_target(i):
    arm_and_takeoff(vehicles[i], 5)

launcher_threads = []
for i in range(0,len(vehicles)):
    print("Launching Vehicle %d:" % (target_systems[i],))
    launcher_threads.append(threading.Thread(target=vehicle_launcher_target,
                                             args=(i,)))
    launcher_threads[-1].start()
for thread in launcher_threads:
    print("Waiting for launcher thread...")
    thread.join()

speed = 2
for i in range(0,len(vehicles)):
    print("Set default/target airspeed to %d on vehicle %d (%s)" % (target_systems[i],speed, str(target_systems[i])))
    vehicles[i].airspeed = speed
    speed += 1

target_lat = 0
target_lon = 0
for i in range(0,len(vehicles)):
    target_lat += vehicles[i].location.global_frame.lat
    target_lon += vehicles[i].location.global_frame.lon

target_lat /= len(vehicles)
target_lon /= len(vehicles)

for i in range(0,len(vehicles)):
    print("vehicle %d going towards first point for 30 seconds ..." % (target_systems[i],))
    point = LocationGlobalRelative(target_lat, target_lon, 20)
    vehicles[i].simple_goto(point)

# sleep so we can see the change in map
def watch_things_for_a_while(duration=30):
    start = time.time()

    while time.time() - start < duration:
        for i in range(0,len(vehicles)):
            vehicle = vehicles[i]
            print("Vehicle %d (%s) Lat=%f Lon=%f MODE=%s" % (target_systems[i], vehicle.mode, vehicle.location.global_frame.lat, vehicle.location.global_frame.lon, vehicle.mode))
            time.sleep(1)

watch_things_for_a_while()

for i in range(0, len(vehicles)):
    if vehicles[i].mode != "RTL":
        print("Vehicle mode should already be in RTL nmode!")

    while vehicles[i].mode != "RTL":
        print("Setting returning to Launch - vehicle %d" % (target_systems[i],))
        vehicles[i].mode = VehicleMode("RTL")
        time.sleep(0.1)

watch_things_for_a_while(duration=30)

print("All done")

if hub_thread is not None:
    hub_should_quit = True
    hub_thread.join()
