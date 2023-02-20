from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import numpy as np
import cv2
import time
import math
from threading import Thread

# tcp:127.0.0.1:5762
# /dev/ttyUSB0


class Drone:

    def __init__(self):

        print("Connecting...")
        self.vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True, baud=57600)
        self.red_exist = 0
        print("Connected.")

    def arm(self):

        while self.vehicle.is_armable == 'False':
            print("Could not provide for ARM.")
            time.sleep(1)

        print("UAV is armable.")
        self.vehicle.mode = VehicleMode("GUIDED")

        while self.vehicle.mode == 'GUIDED':
            print("Switching to GUIDED mode.")
            time.sleep(1)
            break

        self.vehicle.armed = True

        while not self.vehicle.armed:
            print("Waiting for ARM.")
            time.sleep(1)

        print("ARM")

    def take_off(self, target_altitude):

        self.vehicle.simple_takeoff(target_altitude)

        while self.vehicle.location.global_relative_frame.alt <= target_altitude * 0.94:
            print("Altitude: {}".format(self.vehicle.location.global_relative_frame.alt))
            time.sleep(1)

    def read_mission(self, filename):

        wps = open(filename, 'r')
        wp_list = []

        for i in wps:
            try:
                line = i.split(",")
                lat = float(line[0])
                long = float(line[1])
                alt = float(line[2].strip())

                print("Lat:", lat, "Long: ", long, "Alt:", alt)

                cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                              mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                              0,
                              0, 0,
                              0, 0,
                              0, lat, long, alt)

                wp_list.append(cmd)
            except:
                pass

        wps.close()
        return wp_list

    def upload_mission(self, filename):
        mission_list = self.read_mission(filename)

        cmds = self.vehicle.commands
        cmds.clear()
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0,
                    0, 0,
                    0,
                    0, 0, 0, 5))

        for command in mission_list:
            cmds.add(command)

        print('Mission uploaded.')
        self.vehicle.commands.upload()

    def stop_wp(self, wp_no):

        while True:
            next_wp = self.vehicle.commands.next

            if next_wp == wp_no:
                break

    def descend(self, target_alt):

        target_alt = float(target_alt)

        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print("Current altitude:", current_alt, "Speed:0.5")
            self.velocity(0, 0, 0.5)
            time.sleep(1)

            if current_alt < 7:
                break

        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print("Current altitude:", current_alt, "Speed:0.3")
            self.velocity(0, 0, 0.3)
            time.sleep(1)

            if current_alt < target_alt:
                break

    def ascend(self, target_alt):

        target_alt = float(target_alt)

        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt
            print("Current altitude:", current_alt, "Speed:0.3")
            self.velocity(0, 0, -0.3)
            time.sleep(1)

            if current_alt > target_alt:
                break

    def velocity(self, x, y, z, yaw):

        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111,
            0, 0, 0,
            x, y, z,
            0, 0, 0,
            0, 0, 0,
            0, math.radians(yaw))

        self.vehicle.send_mavlink(msg)



    def save_loc(self, file_name):

        red = open(file_name, "w")

        rx = str(self.vehicle.location.global_relative_frame.lat)
        ry = str(self.vehicle.location.global_relative_frame.lon)

        red_loc = (rx + "," + ry + "," + "5")
        red.write(red_loc)

    def set_mid(self, x_lane, y_lane):
        while True:
            if 640 > x_lane > 370 and y_lane < 190:
                self.velocity(0, 0.1, 0, yaw=)
                print("RIGHT")

            elif x_lane < 270 and y_lane < 190:
                self.velocity(0, -0.1, 0, yaw=)
                print("LEFT")

            elif x_lane < 270 and 290 < y_lane < 480:
                self.velocity(0, -0.1, 0, yaw=)
                print("LEFT")

            elif 640 > x_lane > 370 and 480 > y_lane > 290:
                self.velocity(0, 0.1, 0, yaw=)
                print("RIGHT")
                time.sleep(1)

            elif 370 > x_lane > 270 and y_lane < 190:
                self.velocity(0.1, 0, 0, yaw=)
                print("FORWARD")
                time.sleep(1)

            elif 370 > x_lane > 270 and 480 > y_lane > 290:
                self.velocity(-0.1, 0, 0, yaw=)
                print("BACKWARD")
                time.sleep(1)

            elif x_lane < 270 and 290 > y_lane > 190:
                self.velocity(0, -0.1, 0, yaw=)
                print("LEFT")
                time.sleep(1)
            elif 640 > x_lane > 370 and 290 > y_lane > 190:
                self.velocity(0, 0.1, 0, yaw=)
                print("RIGHT")
                time.sleep(1)

            elif 370 > x_lane > 270 and 290 > y_lane > 190:
                self.velocity(0, 0, 0.1, yaw=)
                print("MID")
                time.sleep(1)

    def rectangle(self, mask_color, frame_name):

        contours, hierarchy = cv2.findContours(mask_color, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):

            captured = 0
            area = cv2.contourArea(contour)

            if area > 1000:

                captured += 1

                a, b, c, d = cv2.boundingRect(contour)
                frame_name = cv2.rectangle(frame_name, (a, b), (a + c, b + d), (0, 0, 0), 2)
                cv2.putText(frame_name, "ASOS", (a, b), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 1)

                self.red_exist += 1

                x_mid = a + (c / 2)
                y_mid = b + (d / 2)

                self.set_mid(x_mid, y_mid)

                if captured == 2:
                    break

    def scan(self, low_color, high_color):

        cap = cv2.VideoCapture(0)
        kernel = np.ones((5, 5), "uint8")

        while True:
            _, frame = cap.read()

            frame = cv2.resize(frame, (640, 480))
            frame = cv2.GaussianBlur(frame, (7, 7), 0)
            frame = cv2.erode(frame, kernel, iterations=2)
            frame = cv2.dilate(frame, kernel, iterations=2)

            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv_frame, low_color, high_color)

            self.rectangle(mask, frame)

            cv2.imshow("View", frame)
            cv2.waitKey(10)

            if self.red_exist == 1:

                print("Found red.")

                self.save_loc("Red Location.txt")

                ss = frame
                cv2.imwrite("ScreenShot.jpeg", ss)

                break

        cap.release()
        cv2.destroyAllWindows()




''''
def roll():

    q = quaternion.QuaternionBase([np.radians(45), np.radians(60), np.radians(0)])
    print(q)
    msg = Asena.message_factory.set_attitude_target_encode(
        0,
        0,
        0,
        0b11100010,
        q,
        0,
        0,
        0,
        0)
    Asena.send_mavlink(msg)
    Asena.flush()

'''''
''''
def yaw(yaw_position):
    #Asena.mode = VehicleMode("GUIDED")
    attitude = Asena.attitude
    attitude.yaw = yaw_position
    Asena.send_mavlink(Asena.massage_factory.set_attitude_target_encode(
        0,
        0,
        attitude.roll,
        attitude.pitch,
        yaw_position,
        0,
        0,
        0,
        0,
    ))
'''''

'''''
def send_attitude_target(roll_angle=0.0, pitch_angle=0.0,
                         yaw_angle=None, yaw_rate=0.0, use_yaw_rate=False,
                         thrust=0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = Asena.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = Asena.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        1,  # Target system
        1,  # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle),  # Quaternion
        0,  # Body roll rate in radian
        0,  # Body pitch rate in radian
        math.radians(yaw_rate),  # Body yaw rate in radian/second
        thrust  # Thrust
    )
    Asena.send_mavlink(msg)
    print("send msg")


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]
'''''



