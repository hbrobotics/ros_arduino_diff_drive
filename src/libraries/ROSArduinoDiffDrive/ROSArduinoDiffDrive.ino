/*********************************************************************
 *  ROSArduinoDiffDrive
 
    A simple driver for an Arduino controlled differential drive
    robot.  Assumes use of an Arduino Mega + Pololu motor controller
    shield + Robogaia Mega Encoder shield.

    Target speeds are set by publishing Twist messages on the /cmd_vel
    topic.  Odometry is published on the /odometry_lite topic. Use the
    odomlite_to_odom node to convert to a full Odometry message on the
    /odom topic

    Created for the Pi Robot Project: http://www.pirobot.org

    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Load the custom typedefs for tracking encoder info */
#include "PIDTypes.h"

/* Include the appropriate ROS headers */
#include <ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <ros_arduino_diff_drive/OdometryLite.h>
#include <std_msgs/UInt32.h>

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

/* Define the robot paramters */
int cpr = 8384; // Encoder ticks per revolution for the Pololu 131:1 motor
float wheelDiameter = 0.146; // Scout robot
float baseWidth = 0.291; // Scout robot
float ticksPerMeter = cpr / (PI * wheelDiameter);

unsigned char moving = 0; // is the base in motion?

/* The Pololu motor driver shield */
#include "DualVNH5019MotorShield.h"

/* The Robogaia Mega Encoder shield */
#include "MegaEncoderCounter.h"

/* We need sin and cos for odometry calcuations */
#include <math.h>

/* Create the encoder object */
MegaEncoderCounter encoders(4); // Initializes the Mega Encoder Counter in the 4X Count Mode

/* Create the motor object */
DualVNH5019MotorShield drive;

/* Maximum value for a PWM signal */
#define MAXOUTPUT       255

/* Rate at which encoders are sampled and PID loop is updated */
#define PID_RATE        30     // Hz
const float PID_INTERVAL = 1000.0 / PID_RATE;

/* Odometry publishing rate */
#define ODOM_RATE       10     // Hz
const float ODOM_INTERVAL = 1000.0 / ODOM_RATE;

/* The base and odometry frames */
char baseFrame[] = "/base_link";
char odomFrame[] = "/odom";

/* Counters to track update rates for PID and Odometry */
unsigned long frameTime = 0;
unsigned long nextPID = 0;
unsigned long nextOdom = 0;

/* SetPointInfo struct is defined in PIDTypes.h */
SetPointInfo leftPID, rightPID;

/* OdomInfo struct is defined in PIDTypes.h */
OdomInfo odomInfo;

/* Create the ROS node handle */
ros::NodeHandle nh;

/* A publisher for OdometryLite data on the /odometry_lite topic.
 * Thanks to Austin Hendrix for the code.
 */
ros_arduino_diff_drive::OdometryLite odom_msg;
ros::Publisher odomPub("odometry_lite", &odom_msg);

/* A debugging publisher since nh.loginfo() only takes character constants */
std_msgs::UInt32 log_msg;
ros::Publisher logPub("arduino_log", &log_msg);

/* Convert meters per second to ticks per time frame */
int SpeedToTicks(float v) {
  return int(v * cpr / (PID_RATE * PI * wheelDiameter));
}

/* The callback function to convert Twist messages into motor speeds */
void cmdVelCb(const geometry_msgs::Twist& msg) {
  float x = msg.linear.x; // m/s
  float th = msg.angular.z; // rad/s
  float spd_left, spd_right;

  if (x == 0 && th == 0) {
    moving = 0;
    drive.setSpeeds(0, 0);
    return;
  }

  /* Indicate that we are moving */
  moving = 1;

  if (x == 0) {
    // Turn in place
    spd_right = th * baseWidth / 2.0;
    spd_left = -spd_right;
  } 
  else if (th == 0) {
    // Pure forward/backward motion
    spd_left = spd_right = x;
  } 
  else {
    // Rotation about a point in space
    spd_left = x - th * baseWidth / 2.0;
    spd_right = x + th * baseWidth / 2.0;
  }

  /* Set the target speeds in meters per second */
  leftPID.TargetSpeed = spd_left;
  rightPID.TargetSpeed = spd_right;

  /* Convert speeds to encoder ticks per frame */
  leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
  rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);
}

/* A subscriber for the /cmd_vel topic */
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("/cmd_vel", &cmdVelCb);

/* PID routine to compute the next motor commands */
void doPID(SetPointInfo * p) {
  long Perror;
  long output;

  Perror = p->TargetTicksPerFrame - (p->Encoder - p->PrevEnc);

  // Derivative error is the delta Perror
  output = (Kp * Perror + Kd * (Perror - p->PrevErr) + Ki * p->Ierror) / Ko;
  p->PrevErr = Perror;
  p->PrevEnc = p->Encoder;

  output += p->output;
  // Accumulate Integral error *or* Limit output.
  // Stop accumulating when output saturates
  if (output >= MAXOUTPUT)
    output = MAXOUTPUT;
  else if (output <= -MAXOUTPUT)
    output = -MAXOUTPUT;
  else
    p->Ierror += Perror;

  p->output = output;
}

/* Read the encoder values and call the PID routine */
void updatePID() {
  /* Read the encoders */
  leftPID.Encoder = encoders.YAxisGetCount();
  rightPID.Encoder = encoders.XAxisGetCount();

  /* Record the time that the readings were taken */
  odomInfo.encoderTime = millis();
  odomInfo.encoderStamp = nh.now();

  /* If we're not moving there is nothing more to do */
  if (!moving)
    return;

  /* Compute PID update for each motor */
  doPID(&leftPID);
  doPID(&rightPID);

  /* Set the motor speeds accordingly */
  drive.setSpeeds(leftPID.output, rightPID.output);
}

/* Calculate the odometry update and publish the result */
void updateOdom() {
  double dt, dleft, dright, dx, dy, dxy_ave, dth, vxy, vth;

  /* Get the time in seconds since the last encoder measurement */
  //dt = nh.now().toSec() - odomInfo.lastOdom.toSec();
  dt = (odomInfo.encoderTime - odomInfo.lastEncoderTime) / 1000.0;

  /* Save the encoder time for the next calculation */
  odomInfo.lastEncoderTime = odomInfo.encoderTime;

  /* Calculate the distance in meters traveled by the two wheels */
  dleft = (leftPID.Encoder - odomInfo.prevLeftEnc) / ticksPerMeter;
  dright = (rightPID.Encoder - odomInfo.prevRightEnc) / ticksPerMeter;

  odomInfo.prevLeftEnc = leftPID.Encoder;
  odomInfo.prevRightEnc = rightPID.Encoder;

  /* Compute the average linear distance over the two wheels */
  dxy_ave = (dleft + dright) / 2.0;

  /* Compute the angle rotated */
  dth = (dright - dleft) / baseWidth;

  /* Linear velocity */
  vxy = dxy_ave / dt;

  /* Angular velocity */
  vth = dth / dt;

  /* How far did we move forward? */
  if (dxy_ave != 0) {
    dx = cos(dth) * dxy_ave;
    dy = -sin(dth) * dxy_ave;
    /* The total distance traveled so far */
    odomInfo.linearX += (cos(odomInfo.angularZ) * dx - sin(
    odomInfo.angularZ) * dy);
    odomInfo.linearY += (sin(odomInfo.angularZ) * dx + cos(
    odomInfo.angularZ) * dy);
  }

  /* The total angular rotated so far */
  if (dth != 0)
    odomInfo.angularZ += dth;

  /* Represent the rotation as a quaternion */
  geometry_msgs::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(odomInfo.angularZ / 2.0);
  quaternion.w = cos(odomInfo.angularZ / 2.0);

  /* Publish the distances and speeds on the odom topic. Set the timestamp
   	 to the last encoder time. */
  odom_msg.header.frame_id = odomFrame;
  odom_msg.child_frame_id = baseFrame;
  odom_msg.header.stamp = odomInfo.encoderStamp;
  odom_msg.pose.position.x = odomInfo.linearX;
  odom_msg.pose.position.y = odomInfo.linearY;
  odom_msg.pose.position.z = 0;
  odom_msg.pose.orientation = quaternion;
  odom_msg.twist.linear.x = vxy;
  odom_msg.twist.linear.y = 0;
  odom_msg.twist.linear.z = 0;
  odom_msg.twist.angular.x = 0;
  odom_msg.twist.angular.y = 0;
  odom_msg.twist.angular.z = vth;

  odomPub.publish(&odom_msg);
}

void clearPID() {
  moving = 0;
  leftPID.PrevErr = 0;
  leftPID.Ierror = 0;
  leftPID.output = 0;
  rightPID.PrevErr = 0;
  rightPID.Ierror = 0;
  rightPID.output = 0;
}

void clearAll() {
  clearPID();
  encoders.XAxisReset();
  encoders.YAxisReset();
}

void setup() {
  Serial.begin(57600);
  Serial.println("ROS PID Test");

  /* Intialize the motor driver */
  drive.init();

  /* Set the target speeds in meters per second */
  leftPID.TargetSpeed = 0.0;
  rightPID.TargetSpeed = 0.0;

  /* Convert speeds to encoder ticks per frame */
  leftPID.TargetTicksPerFrame = SpeedToTicks(leftPID.TargetSpeed);
  rightPID.TargetTicksPerFrame = SpeedToTicks(rightPID.TargetSpeed);

  /* Zero out the encoder counts */
  encoders.XAxisReset();
  encoders.YAxisReset();

  /* Initialize the frame time */
  frameTime = 0;

  /* Initialize the ROS node */
  nh.initNode();

  nextPID = PID_INTERVAL;
  nextOdom = ODOM_INTERVAL;

  /* Advertise the Odometry publisher */
  nh.advertise(odomPub);
  //nh.advertise(logPub);

  /* Activate the Twist subscriber */
  nh.subscribe(cmdVelSub);
}

void loop() {
  frameTime = millis();

  /* Is it time for another PID calculation? */
  if (frameTime > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  frameTime = millis();

  /* Is it time for another odometry calculation? */
  if (frameTime > nextOdom) {
    updateOdom();
    nextOdom += ODOM_INTERVAL;
  }

  nh.spinOnce();
}

