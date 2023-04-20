#!/usr/bin/env python3

import time
import weakref
from numbers import Number
from functools import partial
from itertools import repeat, chain
from contextlib import contextmanager
from mavactive import mavactive, mavutil, mavlink

# logging setup
import logging

logging.NOTE = logging.INFO - 5
logging.addLevelName(logging.NOTE, 'NOTE')
class NoteLogger(logging.getLoggerClass()):
    def note(self, msg, *args, **kwargs):
        if self.isEnabledFor(logging.NOTE):
            self._log(logging.NOTE, msg, args, **kwargs)

logging.setLoggerClass(NoteLogger)
logging.note = partial(NoteLogger.note, logging.getLogger())


class Autopilot:
    """ An ArduSub autopilot connection manager. """
    def __init__(self, *args, client=True, thrusters=6, **kwargs):
        self.thrusters = thrusters
        self.master = mavutil.mavlink_connection(*args, **kwargs)

        if client:
            logging.debug('connecting to MAVLink client...')
            self.master.wait_heartbeat()
        else:
            logging.debug('connecting to MAVLink server...')
            self._server_wait_conn()

        logging.info('MAVLink connection successful')

        # send regular heartbeats, as a ground control station
        self.heart = mavactive(self.master, mavlink.MAV_TYPE_GCS)
        self._finalizer = weakref.finalize(self, self.cleanup)

        # convenience
        self.mav        = self.master.mav
        self.recv_match = self.master.recv_match
        self.target     = (self.master.target_system,
                           self.master.target_component)

        logging.debug('checking for autopilot...')
        self._wait_autopilot_heartbeat()
        logging.info('connection to autopilot confirmed')

    def _server_wait_conn(self):
        while 'waiting for server to respond':
            self.master.mav.ping_send(int(time.time() * 1e6), 0, 0, 0)
            if self.master.recv_match():
                break
            time.sleep(0.5)

    def _wait_autopilot_heartbeat(self):
        ensure_autopilot_heartbeat = (
            f'HEARTBEAT.get_srcSystem() == {self.master.target_system} and '
            f'HEARTBEAT.get_srcComponent() == {mavlink.MAV_COMP_ID_AUTOPILOT1}'
        )
        self.recv_match(type='HEARTBEAT', condition=ensure_autopilot_heartbeat,
                        blocking=True)

    def read_param(self, name: str, index: int=-1, timeout: float=None):
        self.mav.param_request_read_send(
            *self.target,
            name.encode('utf8'),
            index
        )
        logging.note(f'read_param({name=}, {index=}, {timeout=})')
        return self.recv_match(type='PARAM_VALUE', blocking=True,
                               timeout=timeout)

    def set_param(self, name: str, value: float, type: int=0,
                  timeout: float=1, retries: int=3):
        name = name.encode('utf8')
        self.mav.param_set_send(
            *self.target,
            name, value, type
        )
        logging.info(f'set_param({name=}, {value=}, {type=})')

        while not (msg := self.recv_match(type='PARAM_VALUE', blocking=True,
                                          timeout=timeout)) and retries > 0:
            retries -= 1
            logging.debug(f'param set timed out after {timeout}s, retrying...')
            self.mav.param_set_send(
                *self.target,
                name, value, type
            )
        return msg

    def __enter__(self):
        ''' Send regular heartbeats while using in a context manager. '''
        logging.info('__enter__ -> reviving heart (if required)')
        self.heart.revive()
        return self

    def __exit__(self, *args):
        ''' Automatically disarm and stop heartbeats on error/context-close. '''
        logging.info('__exit__ -> disarming, and stopping heart')
        self.cleanup(disconnect=False)

    def arm(self):
        self.master.arducopter_arm()
        logging.debug('arm requested, waiting...')
        self.master.motors_armed_wait()
        logging.info('Motors armed!')

    def disarm(self):
        self.master.arducopter_disarm()
        logging.debug('disarm requested, waiting...')
        self.master.motors_disarmed_wait()
        logging.info('Motors disarmed')

    def set_mode(self, mode):
        ''' Sets autopilot 'mode', by name or id number. '''
        if isinstance(mode, str):
            for attempt in range(5):
                if (mapping := self.master.mode_mapping()):
                    break # success - mapping available
                self._wait_autopilot_heartbeat()
                logging.info('mode mapping not available, retrying...')
            else:
                logging.warning('mode change failed - no mapping available!')
                return
            mode_id = mapping[mode.upper()]
        else:
            mode_id = mode
        self.master.set_mode(mode_id)
        logging.debug(f'setting {mode=} ({mode_id=}), waiting...')

        while 'mode change not confirmed':
            ack_msg = self.recv_match(type='COMMAND_ACK', blocking=True)
            # check if acknowledged MAV_CMD_DO_SET_MODE or SET_MODE (old)
            if ack_msg.command in (176, 11):
                if ack_msg.result == 0:
                    logging.info(f'{mode=}, change successful')
                else:
                    result = mavlink.enums['MAV_RESULT'][ack_msg.result]
                    logging.warning('mode change failed!\n\t%s: %s',
                                    result.name, result.description)
                break

    def set_servo(self, servo, pwm):
        ''' Set a single servo output pwm value.

        'servo' can only be outputs that aren't assigned as motors, so is
          generally used for lights/camera etc.

        When in a per_thruster_control context in Servo mode, also allows
          controlling individual thrusters.

        '''
        logging.info(f'set_servo({servo=}, {pwm=})')
        self.master.set_servo(servo, pwm)

    def send_rc(self, rcin1=65535, rcin2=65535, rcin3=65535, rcin4=65535,
                rcin5=65535, rcin6=65535, rcin7=65535, rcin8=65535,
                rcin9=65535, rcin10=65535, rcin11=65535, rcin12=65535,
                rcin13=65535, rcin14=65535, rcin15=65535, rcin16=65535,
                rcin17=65535, rcin18=65535, *, # keyword-only from here
                pitch=None, roll=None, throttle=None, yaw=None, forward=None,
                lateral=None, camera_pan=None, camera_tilt=None, lights1=None,
                lights2=None, video_switch=None):
        ''' Sets all 18 rc channels as specified.

        Values should be between 1100-1900, or left as 65535 to ignore.

        Can specify values:
            positionally,
            or with rcinX (X=1-18),
            or with default RC Input channel mapping names
              -> see https://ardusub.com/developers/rc-input-and-output.html

        It's possible to mix and match specifier types (although generally
          not recommended). Default channel mapping names override positional
          or rcinX specifiers.

        '''
        rc_channel_values = (
            pitch        or rcin1,
            roll         or rcin2,
            throttle     or rcin3,
            yaw          or rcin4,
            forward      or rcin5,
            lateral      or rcin6,
            camera_pan   or rcin7,
            camera_tilt  or rcin8,
            lights1      or rcin9,
            lights2      or rcin10,
            video_switch or rcin11,
            rcin12, rcin13, rcin14, rcin15, rcin16, rcin17, rcin18
        )
        logging.info(f'send_rc')
        logging.debug(rc_channel_values)
        self.mav.rc_channels_override_send(
            *self.target,
            *rc_channel_values
        )

    def clear_motion(self, stopped_pwm=1500):
        ''' Sets all 6 motion direction RC inputs to 'stopped_pwm'. '''
        logging.info('clear_motion')
        self.send_rc(*[stopped_pwm]*6)

    def get_thruster_outputs(self):
        ''' Returns (and notes) the first 'self.thrusters' servo PWM values.

        Offset by 1500 to make it clear how each thruster is active.

        '''
        logging.debug('get_thruster_outputs')
        servo_outputs = self.recv_match(type='SERVO_OUTPUT_RAW',
                                        blocking=True).to_dict()
        thruster_outputs = [servo_outputs[f'servo{i+1}_raw'] - 1500
                            for i in range(self.thrusters)]
        logging.note(f'{thruster_outputs=}')
        return thruster_outputs

    def status_loop(self, duration, delay=0.05):
        ''' Loop for 'duration', with 'delay' between iterations. [s]

        Useful for debugging.

        '''
        start = time.time()
        while time.time() - start < duration:
            self.get_thruster_outputs()
            time.sleep(delay)

    def command_long_send(self, command, confirmation=0, param1=0, param2=0,
                          param3=0, param4=0, param5=0, param6=0, param7=0):
        self.mav.command_long_send(
            *self.target,
            getattr(mavlink, f'MAV_CMD_{command}', command),
            confirmation,
            param1, param2, param3, param4, param5, param6, param7
        )

    def set_message_interval(self, message, interval, response_target=0):
        ''' Set message interval, as per MAV_CMD_SET_MESSAGE_INTERVAL.

        Required to get specific messages if not auto-streamed by connection.

        'message' is the name or id of the message
        'interval' is the desired interval [us]

        '''
        logging.info(f'set_message_interval({message=}, {interval=})')
        self.command_long_send(
            'SET_MESSAGE_INTERVAL',
            param1 = getattr(mavlink, f'MAVLINK_MSG_ID_{message}', message),
            param2 = interval, param7 = response_target)

    def set_bulk_message_intervals(self, messages, interval=None):
        ''' Set several message intervals at once. '''
        if interval is None:
            if isinstance(messages, dict):
                iterator = messages.items()
            else:
                # assume iterator of pairs
                iterator = messages
        elif isinstance(interval, Number):
            iterator = zip(messages, repeat(interval))
        else:
            iterator = zip(messages, interval)

        for message, interval in iterator:
            self.set_message_interval(message, interval)

    def disconnect(self):
        ''' End the MAVLink connection. '''
        logging.info('disconnect -> closing MAVLink connection')
        self.master.close()

    def cleanup(self, *, disconnect=True):
        ''' Attempt to disarm, then stop sending heartbeats.
            Optionally disconnect (true by default).
        '''
        try:
            self.disarm()
        except OSError:
            pass
        self.heart.kill()
        
        if disconnect:
            self.disconnect()

    @classmethod
    def connect_to_client(cls, ip='0.0.0.0', port=14550, *args, **kwargs):
        ''' Default for topside connection to Blue Robotics Companion/BlueOS. '''
        logging.note(f'connect_to_client({ip=}, {port=}, {args=}, {kwargs=})')
        return cls(f'udpin:{ip}:{port}', *args, **kwargs)

    @classmethod
    def connect_to_server(cls, ip='blueos.local', port=14550, *args, **kwargs):
        ''' Less network configuration required, but seemingly less robust. '''
        logging.note(f'connect_to_server({ip=}, {port=}, {args=}, {kwargs=})')
        return cls(f'udpout:{ip}:{port}', *args, **kwargs, client=False)


class ExtendedAutopilot(Autopilot):
    ''' An Autopilot with extended control functionality. '''
    SERVO_OUTPUT_FUNCTIONS = {
        'Disabled': 0,
        'RCPassThru': 1,
        **{f'Motor{i}': param_value for i, param_value in
           enumerate(chain(range(33, 33+8), range(82, 82+4)), start=1)},
        **{f'RCIN{i}': param_value for i, param_value in
           enumerate(range(51, 51+16), start=1)},
    }

    @contextmanager
    def per_thruster_control(self, mode='Servo', stopped_pwm=1500,
                             backup_timeout: float=None,
                             set_timeout: float=1, set_retries: int=3):
        ''' Allow thrusters to be controlled individually, within a context.

        WARNING: while in context, thruster actions no longer require arming,
          and failsafes are no longer engaged!

        'mode' can be 'Servo' (default) or 'RCPassThru' to control thrusters
          either using self.set_servo, or using self.send_rc.

        WARNING: if the vehicle is powered down while in RCPassThru mode the
          thrusters may become active WITHOUT ARMING on next power up, from
          joystick input. It is more powerful than Servo mode because multiple
          thrusters can be controlled in a single function call, but use with
          significant caution.

        For vehicles with more than 6 thrusters RCPassThru mode is not
          recommended, because it doubles-up on existing RC Input mappings
          (e.g. Camera Pan and Tilt are mapped to inputs 7 and 8). That issue
          can be worked around if necessary by mapping individual outputs to
          only otherwise unused RCINx inputs (generally 1-6, 12-18), but that's
          not currently supported/implemented by this function.
        Another alternative is remapping the input channels with PARAM_MAP_RC,
          although that can cause some difficult debugging issues when
          comparing behavior to BR documentation/examples, which assumes the
          default mappings.

        '''
        logging.info('Engaging per-thruster control!')
        logging.warning('Thruster actions no longer require arming!')
        logging.warning('Failsafes no longer engaged!')

        if mode == 'RCPassThru':
            output_function = self.SERVO_OUTPUT_FUNCTIONS[mode]
            logging.critical('RCPassThru mode engaged:\n'
                ' -> THRUSTERS RESPOND DIRECTLY TO RC/JOYSTICK INPUTS')
        elif mode == 'Servo':
            output_function = self.SERVO_OUTPUT_FUNCTIONS['Disabled']
            logging.info('Servo mode engaged:\n'
                         ' -> control thrusters with self.set_servo')
        else:
            raise ValueError(f'Invalid per-thruster control {mode=}')

        self._backup_servo_functions(backup_timeout)
        try:
            # try to make sure thrusters are stopped before changeover
            self._stop_thrusters(mode, stopped_pwm)
            # set first n SERVO outputs to passthrough first n RC inputs
            for n in range(1, self.thrusters + 1):
                self.set_param(f'SERVO{n}_FUNCTION', output_function,
                               timeout=set_timeout, retries=set_retries)
            # make sure thrusters are actually stopped
            self._stop_thrusters(mode, stopped_pwm)

            yield self
        finally:
            # make sure to stop thrusters before changing back
            self._stop_thrusters(mode, stopped_pwm)
            self._restore_servo_functions(set_timeout, set_retries)

        logging.info('per-thruster control successfully exited')
        logging.warning('Arming and failsafe requirements re-engaged')

    def _stop_thrusters(self, mode, stopped_pwm):
        if mode == 'RCPassThru':
            self.send_rc(*[stopped_pwm]*self.thrusters)
        else:
            for servo in range(1, self.thrusters + 1):
                self.set_servo(servo, stopped_pwm)

    def _backup_servo_functions(self, timeout: float=None):
        logging.note('backing up servo thruster functions')
        self._old_servo_functions = {}
        for n in range(1, self.thrusters + 1):
            param = f'SERVO{n}_FUNCTION'
            self._old_servo_functions[param] = self.read_param(param)

    def _restore_servo_functions(self, timeout: float=1, retries: int=3):
        for param, backup in self._old_servo_functions.items():
            self.set_param(param, backup.param_value, backup.param_type,
                           timeout, retries)
        logging.note('servo functions restored')

def configure_logging(level, filename, datefmt):
    # configure logging, based on defaults/user input
    log_level = getattr(logging, level)
    logging.basicConfig(filename=filename, level=log_level, datefmt=datefmt,
                        format=('%(levelname)s: %(asctime)s.%(msecs)03d - '
                                '%(message)s'))

if __name__ == '__main__':
    # running as a script
    # set up some useful commandline arguments
    from argparse import ArgumentParser

    parser = ArgumentParser()
    parser.add_argument('-l', '--log', default='note',
                        help=('Lowest importance level to log '
                              '(debug/note/info/warning/error/critical)'))
    parser.add_argument('-f', '--filename', default='',
                        help=('File to log to - '
                              'prints to console if not specified'))
    parser.add_argument('-d', '--datefmt', default='%H:%M:%S',
                        help='datetime format for logging')
    parser.add_argument('-m', '--mavlink', default='udpin:0.0.0.0:14550',
                        help='MAVLink connection specifier')
    parser.add_argument('-r', '--request', action='store_true',
                        help='Connection requires messages to be requested')
    args=parser.parse_args()
    client = args.mavlink.startswith('udpin')

    configure_logging(args.log.upper(), args.filename, args.datefmt)

    # run/test the desired functionality
    print('If script gets stuck, press CTRL+C to disarm and quit\n')

    with Autopilot(args.mavlink, client=client) as sub:
        if args.request:
            # request SERVO_OUTPUT_RAW at 5Hz
            sub.set_message_interval('SERVO_OUTPUT_RAW', 2e5)

        sub.set_mode('manual')
        sub.get_thruster_outputs()
        # make sure all motion is set to stationary before arming
        sub.clear_motion()
        sub.get_thruster_outputs()
        sub.arm()
        # set some roll
        sub.send_rc(roll=1700)
        sub.status_loop(1)
        # add a bit of throttle
        sub.send_rc(throttle=1600)
        sub.status_loop(1.5)

    # disarm and wait for a bit
    time.sleep(5)

    # re-enter
    with sub:
        logging.info('RE-ENTERING')
        sub.set_mode('stabilize')
        sub.clear_motion()
        sub.arm()
        # set yaw velocity and some forward motion at the same time
        sub.send_rc(yaw=1800, forward=1600)
        sub.status_loop(2)

    # disconnect, and wait for a bit
    sub.disconnect()
    time.sleep(5)

    # make a new connection, and engage per-thruster control mode
    with ExtendedAutopilot(args.mavlink, client=client) as e_sub, \
         e_sub.per_thruster_control():
        # display initial thruster output values
        e_sub.get_thruster_outputs()
        # set some thruster values
        e_sub.set_servo(1, 1300)
        e_sub.status_loop(1)
        e_sub.set_servo(3, 1400)
        e_sub.set_servo(4, 1550)
        e_sub.status_loop(1.5)