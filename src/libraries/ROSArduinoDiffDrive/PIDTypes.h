#ifndef PIDTypes_h
#define PIDTypes_h

#include <ros/time.h>

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
