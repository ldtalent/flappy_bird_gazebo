#! /usr/bin/env python3
"""This script is used to control the flappy bird in Gazebo using arrow keys."""

import rospy
from geometry_msgs.msg import Wrench
from pynput import keyboard
from pynput.keyboard import Key


class KeyDrive():
    """Class to teleoperate the robot using keyboard."""
    def __init__(self):

        update_rate = 50
        freq = 1. / update_rate

        self.fly_force = 7000  # N
        self.dip_force = -5000  # N

        # Publishers
        self.force_pub = rospy.Publisher('/force', Wrench, queue_size=10)

        # Timers
        rospy.Timer(rospy.Duration(freq), self.keyboard_update)

    def fly(self):
        """Fly the bird."""
        force = Wrench()
        force.force.z = self.fly_force
        self.force_pub.publish(force)

    def dip(self):
        """Dip the bird."""
        force = Wrench()
        force.force.z = self.dip_force
        self.force_pub.publish(force)

    def key_press(self, key):
        """Listen for key press."""
        if key == Key.up:
            self.fly()
        return False

    def key_release(self, _):
        """Listen for key release."""
        self.dip()
        return False

    def keyboard_update(self, _):
        """Keyboard Listener for a press and release."""
        with keyboard.Listener(on_press=self.key_press) as listener_for_key_press:
            listener_for_key_press.join()

        with keyboard.Listener(on_release=self.key_release) as listener_for_key_release:
            listener_for_key_release.join()

    def kill_node(self):
        """Function to kill the ROS node."""
        rospy.signal_shutdown("Done")


if __name__ == '__main__':

    rospy.init_node('flyer_node', anonymous=True)
    KeyDrive()
    rospy.spin()
