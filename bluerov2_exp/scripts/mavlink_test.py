# from pymavlink import mavutil
# import time

# # Connect to the autopilot via serial
# connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# connection.wait_heartbeat()
# print("Connected to the vehicle")

# # Set the servo pin (e.g., 9 for AUX OUT 1)
# servo_pin = 0

# # Function to send PWM
# def send_pwm(pin, pwm_value):
#     connection.mav.command_long_send(
#         connection.target_system,
#         connection.target_component,
#         mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
#         0,
#         pin,
#         pwm_value,
#         0, 0, 0, 0, 0
#     )
#     print(f"Sent PWM {pwm_value} to servo pin {pin}")

# # Sweep PWM from 800 to 2200 and back
# try:
#     while True:
#         # Increasing PWM: 800 -> 2200
#         for pwm in range(800, 2201, 100):  # Step by 100 for smoother sweep
#             send_pwm(servo_pin, pwm)
#             time.sleep(0.1)  # Delay for smooth transition

#         # Decreasing PWM: 2200 -> 800
#         for pwm in range(2200, 799, -100):
#             send_pwm(servo_pin, pwm)
#             time.sleep(0.1)

# except KeyboardInterrupt:
#     print("\nPWM sweep stopped by user")
#     # Optionally reset PWM to neutral
#     send_pwm(servo_pin, 1500)



from pymavlink import mavutil

# Connect to the MAVLink vehicle
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Connected to the vehicle")

# RC Channels: Typically channels 1-8, adjust as needed
channels = [1500] * 8  # Neutral values for all channels

# Override channel 6 with a new PWM value
channels[0] = 1700  # Example PWM value for channel 6 (5 index in Python)


# Sweep PWM from 800 to 2200 and back
try:
    while True:
        # Increasing PWM: 800 -> 2200
        
        for pwm in range(800, 2201, 100):  # Step by 100 for smoother sweep
            # Send RC override
            channels[0] = pwm
            connection.mav.rc_channels_override_send(
                connection.target_system,
                connection.target_component,
                *channels
            )

        # Decreasing PWM: 2200 -> 800
        for pwm in range(2200, 799, -100):
            channels[0] = pwm
            connection.mav.rc_channels_override_send(
                connection.target_system,
                connection.target_component,
                *channels
            )
            # send_pwm(servo_pin, pwm)
            # time.sleep(0.1)
            
except KeyboardInterrupt:
    print("\nPWM sweep stopped by user")


print("RC channel override sent")
