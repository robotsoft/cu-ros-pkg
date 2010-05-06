#ifndef IROBOT_CREATE_H
#define IROBOT_CREATE_H

#include "ros/console.h"
#include "serial_port/lightweightserial.h"
#include <time.h>
#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <vector>

using namespace std;

typedef struct Packet {
  uint8_t opcode;     // opcode to request sensor packet
  int numBytes;       // 1 or 2
  uint8_t data[2];    // length = numBytes, data[0] = highest byte
} Packet;

class IRobotCreate
{
 public:
  IRobotCreate();
  ~IRobotCreate();
  //x: left - right +; y: back - front +; theta: CCW + CW -
  bool getOdometricPos(double &x, double &y, double &th);   // get x (mm), y (mm), and theta (deg) displacement from initial pos
  bool getBumper(int &hit);
  bool getBatt(int &b);
  bool setVelocity(double x_vel, double w_vel);             // linear, angular velocities	
  bool setWheels(double l_vel, double r_vel);               //left and right wheel velocities
  bool Start();
  bool FullControl();
  bool Dock();
  bool playSong(int num);
  bool resetDisplacements();                                // set x, y, and th displacements to 0
  bool streamData(vector<Packet> &packets);
  bool updateStates();                               // updates x, y, and th - call this often 
 private:
  bool setSong();
  void clearAllReading();
  void streamSensors(vector<Packet> &packets);
  void querySensors(vector<Packet> &packets);
  void Wait();		//10 ms
  void SRead(uint8_t * toRead);
  void SWrite(uint8_t toWrite);
  static void toOpcode(int decimal, int & high, int & low); // convert int to 2 uint8_t
  static int toDecimal(uint8_t high, uint8_t low);          // convert 2 uint8_t tp int

  LightweightSerial *serial_port;
  vector<Packet> sensorPackets;                             // used to request distance, angle, bumper and battery sensor data
  double x, y, th;                                          // cummulative x, y, and th displacements from start
  int bump;
  int batt;
};

#define CREATE_OPCODE_START            128
#define CREATE_OPCODE_BAUD             129
#define CREATE_OPCODE_SAFE             131
#define CREATE_OPCODE_FULL             132
#define CREATE_OPCODE_SPOT             134
#define CREATE_OPCODE_COVER            135
#define CREATE_OPCODE_DEMO             136
#define CREATE_OPCODE_DRIVE            137
#define CREATE_OPCODE_MOTORS           138
#define CREATE_OPCODE_LEDS             139
#define CREATE_OPCODE_SONG             140
#define CREATE_OPCODE_PLAY             141
#define CREATE_OPCODE_SENSORS          142
#define CREATE_OPCODE_COVERDOCK        143
#define CREATE_OPCODE_PWM_MOTORS       144
#define CREATE_OPCODE_DRIVE_WHEELS     145
#define CREATE_OPCODE_DIGITAL_OUTPUTS  147
#define CREATE_OPCODE_STREAM           148
#define CREATE_OPCODE_QUERY_LIST       149
#define CREATE_OPCODE_DO_STREAM        150
#define CREATE_OPCODE_SEND_IR_CHAR     151
#define CREATE_OPCODE_SCRIPT           152
#define CREATE_OPCODE_PLAY_SCRIPT      153
#define CREATE_OPCODE_SHOW_SCRIPT      154
#define CREATE_OPCODE_WAIT_TIME        155
#define CREATE_OPCODE_WAIT_DISTANCE    156
#define CREATE_OPCODE_WAIT_ANGLE       157
#define CREATE_OPCODE_WAIT_EVENT       158


#define CREATE_DELAY_MODECHANGE_MS   9000

#define CREATE_MODE_OFF                  0
#define CREATE_MODE_PASSIVE              1
#define CREATE_MODE_SAFE                 2
#define CREATE_MODE_FULL                 3

#define CREATE_TVEL_MAX_MM_S           350
  
#define CREATE_RADIUS_MAX_MM         32767
#define CREATE_RADIUS_STRAIGHT       32768
#define CREATE_RADIUS_TURN_CW        65535
#define CREATE_RADIUS_TURN_CCW           1

#define CREATE_SENSOR_PACKET_SIZE       26

#define CREATE_CHARGING_NOT              0
#define CREATE_CHARGING_RECOVERY         1
#define CREATE_CHARGING_CHARGING         2
#define CREATE_CHARGING_TRICKLE          3
#define CREATE_CHARGING_WAITING          4
#define CREATE_CHARGING_ERROR            5
#define CREATE_SENSOR_BUMPER             7
#define CREATE_SENSOR_DISTANCE		19
#define CREATE_SENSOR_ANGLE		20
#define CREATE_BATT                     25
#define CREATE_OI_MODE                  35
#define CREATE_REQUESTED_VELOCITY	39
#define CREATE_REQUESTED_RADIUS		40
#define CREATE_REQUESTED_RVELOCITY	41
#define CREATE_REQUESTED_LVELOCITY	42
//in unit of meter
#define CREATE_AXLE_LENGTH           0.258

#define CREATE_DIAMETER               0.26
#define CREATE_RADIUS                 0.13
#define CREATE_BUMPER_XOFFSET         0.05
#define CREATE_BATT_CAPACITY          3300
#endif

