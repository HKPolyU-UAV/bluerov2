#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses

import rospy
from sensor_msgs.msg import Joy


class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class SimpleKeyTeleop():
    def __init__(self, interface):
        rospy.init_node('key_teleop')
        self._interface = interface
        self._pub_cmd = rospy.Publisher('joy', Joy)
        self._hz = rospy.get_param('~hz', 10)
        self._last_pressed = {}
        self._gain = rospy.get_param('~gain', 0.25)
        self._axes_bindings = {ord('w'): (0, 1.0),
                                   ord('s'): (0, -1.0),
                                   ord('a'): (1, 1.0),
                                   ord('d'): (1, -1.0),
                                   ord(' '): (2, 1.0),
                                   ord('c'): (2, -1.0),
                                   curses.KEY_LEFT: (3, 1.0),
                                   curses.KEY_RIGHT: (3, -1.0)}
        self._button_bindings = {ord('1'): 0,
                                 ord('2'): 1,
                                 ord('3'): 2,
                                 ord('4'): 3,
                                 ord('5'): 4}
        self._axes = [0.0]*4
        self._buttons = [0]*5

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_joy()
            self._publish()
            rate.sleep()

    def _set_joy(self):
        stamp = rospy.Time.now()
        keys = []
        self._buttons = [0]*5
        for a in self._last_pressed:
            if (stamp - self._last_pressed[a]).to_sec() < 0.05:
                keys.append(a)
        for k in keys:
            if k in self._axes_bindings:
                i, gain = self._axes_bindings[k]
                self._axes[i] = self._axes[i] + gain*self._gain
                self._axes[i] = 0.0 if abs(self._axes[i]) < self._gain else self._axes[i]
            else:
                self._buttons[self._button_bindings[k]] = 1
        self._axes = [min(1.0, a) for a in self._axes]
        self._axes = [max(-1.0, a) for a in self._axes]

    def _get_joy(self):
        joy = Joy()
        joy.header.frame_id = 'keyboard'
        joy.header.stamp = rospy.Time.now()
        joy.axes = self._axes
        joy.buttons = self._buttons
        return joy

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self._axes_bindings:
            self._last_pressed[keycode] = rospy.Time.now()
        elif keycode in self._button_bindings:
            self._last_pressed[keycode] = rospy.Time.now()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Surge: {}, Sway: {}, Heave: {}, Yaw: {}'.format(*self._axes))
        self._interface.write_line(5, 'X +/-: w/s, Y +/-: a/d, Z +/-: space/c, PSI +/-: LEFT/RIGHT')
        self._interface.write_line(7, 'Arm: 1, Disarm: 2, Manual: 3, Depth: 4, Stabilize: 5')
        self._interface.refresh()
        joy = self._get_joy()
        self._pub_cmd.publish(joy)


def main(stdscr):
    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass