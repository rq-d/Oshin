#!/usr/bin/env python3

from pymavlink import mavutil
import time

# Send velocity command to move forward (x-direction)
def send_ned_velocity(vx, vy, vz, duration):
    for _ in range(duration):
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

if __name__=="__main__":

  IpPort = 'udp:127.0.0.1:14550'
  master = mavutil.mavlink_connection(IpPort)
  print("Connecting to ", IpPort)

  # Wait for the heartbeat
  master.wait_heartbeat()
  print("Heartbeat received")

  # Set mode to GUIDED
  # master.set_mode_auto()
  master.set_mode("GUIDED")

  # Arm the vehicle
  master.mav.command_long_send(
      master.target_system,
      master.target_component,
      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
      0,
      1, 0, 0, 0, 0, 0, 0
  )

  # Wait until armed
  master.motors_armed_wait()
  print("Armed!")

  # Take off to 500 meters
  master.mav.command_long_send(
      master.target_system,
      master.target_component,
      mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
      0,
      0, 0, 0, 0, 0, 0, 30
  )
  print("Taking off")

  # send_ned_velocity(1.0, 0, 0, 5)  # Move forward at 1 m/s for 5 seconds
  # print("Moving forward...")

#   # Land
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_NAV_LAND,
#     0,
#     0, 0, 0, 0, 0, 0, 0
# )

# print("Landing...")
# time.sleep(10)

# # Disarm
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0,
#     0, 0, 0, 0, 0, 0, 0
# )

# print("Disarmed and complete.")
