import time
import sys
from pymavlink import mavutil


class bluerov2_mavlink():
    '''
    Initialization, set udpin and wait for hearbeat
    the default connection is available at ip 192.168.2.1 and port 14550
    '''
    def __init__(self):
        self.master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

        # Make sure the connection is valid
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (self.master.target_system, self.master.target_component))

    '''
    Arm or disarm the vehicle
    if arm = 0, disarm the vehicle, if arm = 1, arm the vehicle
    '''
    def arm_disarm_vehicle(self,arm):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            arm, 0, 0, 0, 0, 0, 0)
        if arm == 0:
            print("Waiting for the vehicle to disarm")
            self.master.motors_disarmed_wait()
            print("BlueROV2 Disarmed!")
        elif arm == 1:
            print("Waiting for the vehicle to arm")
            self.master.motors_armed_wait()
            print('BlueROV2 Armed!')

    '''
    Directly control a Pixhawk servo output with pymavlink
    There are 8 main outputs and 6 auxiliary outputs on Pixhawk
    servos are connected to auxiliart outputs
    '''
    def set_servo_pwm(self, servo_n, microseconds):
        # master.set_servo(servo_n+8, microseconds)
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,            # first transmission of this command
            servo_n + 8,  # servo instance, offset by 8 MAIN outputs
            microseconds, # PWM pulse-width
            0,0,0,0,0     # unused parameters
        )

    '''
    Moves gimbal to given position
    Args:
        tilt (float): tilt angle in centidegrees (0 is forward)
        roll (float, optional): roll angle in centidegrees (0 is forward)
        pan  (float, optional): pan angle in centidegrees (0 is forward)
    '''
    def set_camera_gimbal(self, tilt, roll=0, pan=0):
        self.master.mav.command_long_send(
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_CMD_DO_MOUNT_CONTROL,
        1,
        tilt,
        roll,
        pan,
        0, 0, 0,
        mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING)

    def request_parameter(self):
        # Request all parameters
        self.master.mav.param_request_list_send(
            self.master.target_system, self.master.target_component
        )
        while True:
            time.sleep(0.01)
            try:
                message = self.master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
                print('name: %s\tvalue: %d' % (message['param_id'],
                                       message['param_value']))
            except Exception as error:
                print(error)
                sys.exit(0)
if __name__ == '__main__':
    bluerov2 = bluerov2_mavlink()
    #bluerov2.set_servo_pwm(1,1700)
    #bluerov2.arm_disarm_vehicle(1)
    
    for us in range(1100, 1900, 50):
        bluerov2.set_servo_pwm(1, us)
        time.sleep(0.125)
        print("set pwm:",us)
    
    #bluerov2.request_parameter()
    #bluerov2.arm_disarm_vehicle(0)

