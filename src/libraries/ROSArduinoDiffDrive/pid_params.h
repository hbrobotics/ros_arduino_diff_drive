#ifndef pid_params_h
#define pid_params_h

#include <ros/time.h>

/* PID Parameters */
int Kp = 20;
int Kd = 12;
int Ki = 0;
int Ko = 50;

/* Define the robot paramters */
int cpr = 8384; // Encoder ticks per revolution for the Pololu 131:1 motor
float wheelDiameter = 0.146; // meters
float wheelTrack = 0.291; // meters
float ticksPerMeter = cpr / (PI * wheelDiameter);

/* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
#define AUTO_STOP_INTERVAL 2000
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Setpoint Info For a Motor */
typedef struct {
  double TargetSpeed;            // target speed in m/s
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count
  int PrevErr;                   // last error
  int Ierror;                    // integrated error
  int output;                    // last motor setting
}
SetPointInfo;

/* A struct to hold Odometry info */
typedef struct {
  ros::Time lastOdom;            // last ROS time odometry was calculated
  ros::Time encoderStamp;	 // last ROS time encoders were read
  unsigned long encoderTime;     // most recent millis() time encoders were read
  unsigned long lastEncoderTime; // last millis() time encoders were read
  unsigned long lastOdomTime;    // last millis() time odometry was calculated
  long prevLeftEnc;              // last left encoder reading used for odometry
  long prevRightEnc;             // last right encoder reading used for odometry
  float linearX;	         // total linear x distance traveled
  float linearY;	         // total linear y distance traveled
  float angularZ;		 // total angular distance traveled
}
OdomInfo;

#endif
