A simple driver for an Arduino controlled differential drive robot.
Uses rosserial and assumes the use of an Arduino Mega + Pololu motor
controller shield + Robogaia Mega Encoder shield.

Installation
============

$ roscd ~/ros_workspace

$ git clone https://github.com/hbrobotics/ros_arduino_diff_drive.git

$ cd ros_arduino_diff_drive

$ rosmake

$ rosrun rosserial_client make_library.py <sketchbook>/libraries ros_arduino_diff_drive

where <sketchbook> is the path to your Arduino sketchbook directory.

$ roscd ros_arduino_diff_drive/src/libraries

$ cd <sketchbook>

$ ln -s `rospack find ros_arduino_diff_drive/src/libraries/ROSArduinoDiffDrive` ./ROSArduinoDiffDrive






