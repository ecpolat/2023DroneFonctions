import ASOS
import numpy as np
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil

low_red = np.array([14, 164, 0])
high_red = np.array([28, 255, 255])

drone = ASOS.Drone()
drone.upload_mission("ddd.txt")


while True:

    print(" ch8 : %s " % drone.vehicle.channels['8'])
    s = drone.vehicle.channels['8']
    if s < 1500:
        drone.vehicle.mode = VehicleMode("STABILIZE")
        print("STABILIZE")
        time.sleep(2)

    if s > 1900:

        drone.arm()
        drone.take_off(5)
        drone.vehicle.mode = 'AUTO'
        drone.stop_wp(1)
        drone.vehicle.mode = 'GUIDED'
        drone.ascend(6)
        drone.scan(low_red, high_red)
        drone.vehicle.mode = 'AUTO'
        drone.stop_wp(3)
        drone.vehicle.mode = 'LAND'
        break
