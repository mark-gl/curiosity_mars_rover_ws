#!/usr/bin/env python3

# This file has been modified from the original teleop_twist_keyboard.py file.
# It can be found here on GitHub:
# https://github.com/ros-teleop/teleop_twist_keyboard

from __future__ import print_function
import threading
import sys
import select
import termios
import tty
import rospy
from curiosity_mars_rover_control.srv import Mast, MastRequest

msg = """
Reading from the keyboard and publishing to the mast!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
n   : raise/lower mast
q/z : increase/decrease both speeds by 10%
w/x : increase/decrease only vertical speed by 10%
e/c : increase/decrease only horizontal speed by 10%
CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}


class PublishThread(threading.Thread):
    def __init__(self, rate, service):
        super(PublishThread, self).__init__()
        self.mast_service = service
        self.x = 0.0
        self.y = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.inactive = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0)
        self.join()

    def run(self):
        # twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)
            self.condition.release()
            mast_request = MastRequest()
            mast_request.mode = "rotate"
            mast_request.rot_x = self.x
            mast_request.rot_y = self.y
            result = self.mast_service(mast_request)
            if not result.success and not self.inactive:
                self.inactive = True


def toggle_mast(pub_thread):
    mast_request = MastRequest()
    mast_request.mode = "toggle"
    result = mast_service(mast_request)
    print("Toggling mast...")
    if not result.success:
        print("Couldn't raise/lower the mast!")
    else:
        print(result.status_message)
        pub_thread.inactive = False


def get_key(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "current speed:\tvertical %s\thorizontal %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_keyboard_mast_node')
    rospy.wait_for_service('/curiosity_mars_rover/mast_service')

    speed = 0.01
    turn = 0.01

    mast_service = rospy.ServiceProxy(
        '/curiosity_mars_rover/mast_service', Mast)
    pub_thread = PublishThread(0.0, mast_service)

    x = 0
    y = 0
    status = 0

    try:
        pub_thread.update(x, y, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while(1):
            key = get_key(None)
            if (key in moveBindings.keys() or key in speedBindings.keys()) and pub_thread.inactive:
                print(
                    "Can't teleoperate mast as it is not raised. Press 'n' to raise the mast.")
            elif key in moveBindings.keys():
                x = -1 * moveBindings[key][0] * speed
                y = moveBindings[key][1] * turn
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == 'n':
                toggle_mast(pub_thread)
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and x == 0 and y == 0:
                    continue
                x = 0
                y = 0
                if (key == '\x03'):
                    break
            pub_thread.update(x, y, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
