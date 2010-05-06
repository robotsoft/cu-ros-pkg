#include <cstdlib>
#include <math.h>
#include "ros/console.h"
#include "irobot_create/irobot_create.h"

IRobotCreate::IRobotCreate()
  : x(0), y(0), th(0), bump(0), batt(0)
{
  // see if the IROBOT_CREATE_OPPORT variable is defined
  const char *default_port = "/dev/ttyUSB0";
  const char *port_env = getenv("IROBOT_CREATE_PORT");
  const char *serial_port_str = (port_env ? port_env : default_port);
  ROS_DEBUG("irobotcreate ctor, serial_port_str = [%s]", serial_port_str);
  serial_port = new LightweightSerial(serial_port_str, 57600);

  // initialize distance and angle sensor packets
  Packet distPacket, angPacket, bumpPacket, battPacket; //for alignment purpose

  distPacket.opcode = CREATE_SENSOR_DISTANCE;
  distPacket.numBytes = 2;
  distPacket.data[0] = distPacket.data[1] = (uint8_t) 0;
  battPacket = bumpPacket = angPacket = distPacket; 
  angPacket.opcode = CREATE_SENSOR_ANGLE;
  bumpPacket.opcode = CREATE_SENSOR_BUMPER;
  bumpPacket.numBytes = 1;
  battPacket.opcode = CREATE_BATT;
  sensorPackets.push_back(distPacket);
  sensorPackets.push_back(angPacket);
  sensorPackets.push_back(bumpPacket);
  sensorPackets.push_back(battPacket);
}

IRobotCreate::~IRobotCreate()
{
  ROS_DEBUG("irobotcreate dtor");
  ROS_DEBUG("closing serial port");
  delete serial_port;
  serial_port = NULL;
  ROS_DEBUG("irobotcreate dtor complete");
}


bool IRobotCreate::resetDisplacements()
{
  x = y = th = 0;
  //clearAllReading();
  return true;
}

void IRobotCreate::Wait() {
  usleep(CREATE_DELAY_MODECHANGE_MS);
}

void IRobotCreate::SWrite(uint8_t toWrite) {
  while (true)
    if (serial_port->write(toWrite)) break;
}

void IRobotCreate::SRead(uint8_t * toRead) {
  while (true)
    if (serial_port->read(toRead)) break;
}

void IRobotCreate::clearAllReading() {
  uint8_t * garbage = (uint8_t *)malloc(2000);
  serial_port->read_block(garbage, 2000);
  free(garbage);
}

bool IRobotCreate::Start() {
  serial_port->write((uint8_t)CREATE_OPCODE_START);
  Wait();
  
  FullControl();
  //clearAllReading();
  //Wait();
  
  setSong();
  ROS_DEBUG("irobotcreate started");
  //streamSensors(odometryPackets);
  return true;
}

bool IRobotCreate::FullControl() {
  SWrite((uint8_t)CREATE_OPCODE_FULL);
  return true;
}

bool IRobotCreate::Dock() {
  SWrite((uint8_t)CREATE_OPCODE_COVERDOCK);
  return true;
}

//right now hard-coded to the Mario song, ported from player driver
bool IRobotCreate::setSong() {
  unsigned char length = 75;
  unsigned char notes[length];
  unsigned char note_lengths[length];
  int count = 0;
  notes[count] = 76;	note_lengths[count++] = 8;
  notes[count] = 76;	note_lengths[count++] = 16;
  notes[count] = 76;	note_lengths[count++] = 16;
  notes[count] = 72;	note_lengths[count++] = 8;
  notes[count] = 76;	note_lengths[count++] = 16;
  notes[count] = 79;	note_lengths[count++] = 32;
  notes[count] = 67;	note_lengths[count++] = 32;
  //7
  notes[count] = 72;	note_lengths[count++] = 24;
  notes[count] = 67;	note_lengths[count++] = 24;	
  notes[count] = 64;	note_lengths[count++] = 24;
  notes[count] = 69;	note_lengths[count++] = 16;	
  notes[count] = 71;	note_lengths[count++] = 16;
  notes[count] = 70;	note_lengths[count++] = 8;
  notes[count] = 69;	note_lengths[count++] = 16;
  //14
  notes[count] = 67;	note_lengths[count++] = 8;
  notes[count] = 72;	note_lengths[count++] = 16;	
  notes[count] = 76;	note_lengths[count++] = 8;
  notes[count] = 81;	note_lengths[count++] = 16;	
  notes[count] = 77;	note_lengths[count++] = 8;
  notes[count] = 79;	note_lengths[count++] = 16;
  notes[count] = 76;	note_lengths[count++] = 16;
  //21
  notes[count] = 72;	note_lengths[count++] = 8;
  notes[count] = 74;	note_lengths[count++] = 16;
  notes[count] = 71;	note_lengths[count++] = 24;
  //24
  
  notes[count] = 60;	note_lengths[count++] = 16;	
  notes[count] = 79;	note_lengths[count++] = 8;
  notes[count] = 78;	note_lengths[count++] = 8;
  notes[count] = 77;	note_lengths[count++] = 8;
  notes[count] = 74;	note_lengths[count++] = 8;
  notes[count] = 75;	note_lengths[count++] = 8;
  notes[count] = 76;	note_lengths[count++] = 16;
  //31
  
  notes[count] = 67;	note_lengths[count++] = 8;
  notes[count] = 69;	note_lengths[count++] = 8;
  notes[count] = 72;	note_lengths[count++] = 16;
  notes[count] = 69;	note_lengths[count++] = 8;
  notes[count] = 72;	note_lengths[count++] = 8;
  notes[count] = 74;	note_lengths[count++] = 8;
  //37
	
  notes[count] = 60;	note_lengths[count++] = 16;	
  notes[count] = 79;	note_lengths[count++] = 8;
  notes[count] = 78;	note_lengths[count++] = 8;
  notes[count] = 77;	note_lengths[count++] = 8;
  notes[count] = 74;	note_lengths[count++] = 8;
  notes[count] = 75;	note_lengths[count++] = 8;
  notes[count] = 76;	note_lengths[count++] = 16;
  //44
  notes[count] = 76;	note_lengths[count++] = 4;
  notes[count] = 78;	note_lengths[count++] = 4;
  notes[count] = 84;	note_lengths[count++] = 16;
  notes[count] = 84;	note_lengths[count++] = 8;
  notes[count] = 84;	note_lengths[count++] = 16;
  notes[count] = 84;	note_lengths[count++] = 16;
  //50	
  notes[count] = 60;	note_lengths[count++] = 16;	
  notes[count] = 79;	note_lengths[count++] = 8;
  notes[count] = 78;	note_lengths[count++] = 8;
  notes[count] = 77;	note_lengths[count++] = 8;
  notes[count] = 74;	note_lengths[count++] = 8;
  notes[count] = 75;	note_lengths[count++] = 8;
  notes[count] = 76;	note_lengths[count++] = 16;
  //57
	
  notes[count] = 67;	note_lengths[count++] = 8;
  notes[count] = 69;	note_lengths[count++] = 8;
  notes[count] = 72;	note_lengths[count++] = 16;
  notes[count] = 69;	note_lengths[count++] = 8;
  notes[count] = 72;	note_lengths[count++] = 8;
  notes[count] = 74;	note_lengths[count++] = 16;
  //63
  
  notes[count] = 70;	note_lengths[count++] = 4;
  notes[count] = 72;	note_lengths[count++] = 4;
  notes[count] = 75;	note_lengths[count++] = 16;
  notes[count] = 69;	note_lengths[count++] = 4;
  notes[count] = 71;	note_lengths[count++] = 4;
  notes[count] = 74;	note_lengths[count++] = 16;
  notes[count] = 67;	note_lengths[count++] = 4;
  notes[count] = 69;	note_lengths[count++] = 4;
  notes[count] = 72;	note_lengths[count++] = 16;
  notes[count] = 67;	note_lengths[count++] = 8;
  notes[count] = 67;	note_lengths[count++] = 16;
  notes[count] = 60;	note_lengths[count++] = 24;
  /*
  serial_port->write((uint8_t)CREATE_OPCODE_SONG);
  Wait();
  serial_port->write((uint8_t)1);
  Wait();
  serial_port->write((uint8_t)length);
  Wait();
  */
  SWrite((uint8_t)CREATE_OPCODE_SONG);
  SWrite((uint8_t)1);
  SWrite((uint8_t)length);
  for (int i=0; i<length; i++) {
    /*
    serial_port->write((uint8_t)notes[i]); Wait();
    serial_port->write((uint8_t)note_lengths[i]); Wait();
    */
    SWrite((uint8_t)notes[i]);
    SWrite((uint8_t)note_lengths[i]);
  }
  return true;
}

bool IRobotCreate::playSong(int num) {
  if (num>=0 && num <16) {
    /*
    serial_port->write((uint8_t)CREATE_OPCODE_PLAY);
    Wait();
    serial_port->write((uint8_t)num);
    Wait();*/
    SWrite((uint8_t)CREATE_OPCODE_PLAY);
    SWrite((uint8_t)num);
    return true;
  }
  return false;
}

bool IRobotCreate::getOdometricPos(double &x, double &y, double &th)
{
  x = this->x / 1000.0;
  y = this->y / 1000.0;
  th = this->th;
  return true;
}
bool IRobotCreate::getBumper(int &hit)
{
  hit = bump;
  return true;
}

bool IRobotCreate::getBatt(int &b)
{
  float ptg = (float)batt / (float)CREATE_BATT_CAPACITY * 100.0;
  b = (int)ptg;
  return true;
}

// don't call this method too often, cos the angle resolution is low
bool IRobotCreate::updateStates()
{
  //if (streamData(odometryPackets)) {
  querySensors(sensorPackets);
  Packet &distPacket = sensorPackets[0];
  Packet &angPacket = sensorPackets[1];
  Packet &bumpPacket = sensorPackets[2];
  Packet &battPacket = sensorPackets[3];
  double dist = (double)toDecimal(distPacket.data[0], distPacket.data[1]);
  int dth = toDecimal(angPacket.data[0], angPacket.data[1]);
  double dth_rad = (double)dth / 180.0 * M_PI; // convert the angle from degree to rad
  
  if (dth != 0) {
    double radius = dist / dth_rad;
    if (dist > 0.0) dist = dist - radius * 2.0 * M_PI * (int)(dth_rad / 2.0 / M_PI + 0.5);
    else dist = dist + radius * 2.0 * M_PI * (int)(dth_rad / 2.0 / M_PI - 0.5);
    dist = dist * 2.0 / dth_rad * sin (dth_rad / 2.0);
  }
  
  x += dist * cos(th + dth_rad/2.0);
  y += dist * sin(th + dth_rad/2.0);
  th += dth_rad;
  bump = bumpPacket.data[0] & 3;
  batt = toDecimal(battPacket.data[0], battPacket.data[1]);
  return true;
}

void IRobotCreate::streamSensors(vector<Packet> &packets)
{
  serial_port->write((uint8_t)CREATE_OPCODE_STREAM);
  Wait();
  serial_port->write((uint8_t)packets.size());
  Wait();
  for (unsigned i = 0; i < packets.size(); ++i) {
    serial_port->write(packets[i].opcode);
    Wait();    
  }
}

bool IRobotCreate::streamData(vector<Packet> &packets)
{
  uint8_t header = 0, num_bytes = 0, check_sum = 0, sum = 0;
  while (header != 19 || num_bytes != 6) {
    serial_port->read(&header);
    Wait();
    serial_port->read(&num_bytes);
    Wait();
  }

  sum = num_bytes + header;
  /*
  for (int i =0; i< num_bytes; i++) {
    uint8_t data;
    serial_port->read(&data);
    sum += data;
    Wait();
    }*/
  printf("%d %d ", header, num_bytes);
  for (unsigned i = 0; i < packets.size(); ++i) {
    uint8_t ID;
    serial_port->read(&ID);
    Wait();
    sum += ID;
    printf("%d ", ID);
    for (int j = 0; j < packets[i].numBytes; ++j) {
      serial_port->read(&packets[i].data[j]);
      Wait();
      sum += packets[i].data[j];
      printf("%d ", packets[i].data[j]);
    }
  }
  serial_port->read(&check_sum);
  Wait();
  sum += check_sum;
  printf("%d =%d\n", check_sum, sum);
  if (sum == 0) return true;
  return false;
}

// call this method at least once a minute, cos max dist_x stored is 65536
void IRobotCreate::querySensors(vector<Packet> &packets)
{
  // request packets
  //serial_port->write((uint8_t)CREATE_OPCODE_QUERY_LIST);
  //WaitWrite();
  SWrite((uint8_t)CREATE_OPCODE_QUERY_LIST);
  //serial_port->write((uint8_t)(packets.size() + 1)); //for alignment purpose, check OI mode
  //WaitWrite();
  SWrite((uint8_t)(packets.size() + 1));
  //serial_port->write((uint8_t)CREATE_OI_MODE);
  //WaitWrite();
  SWrite((uint8_t)CREATE_OI_MODE);
  for (unsigned i = 0; i < packets.size(); ++i) {
    //serial_port->write(packets[i].opcode); 
    //WaitWrite();
    SWrite(packets[i].opcode);
  }
  /*
  uint8_t *array = (uint8_t *)malloc(sizeof(uint8_t) * 10);
  serial_port->read_block(array, 10);
  for (int i=0; i< 10; i++)
    printf("%d ", array[i]);
  printf("\n");
  */
  // for alignment purpose
  uint8_t oi_mode = 0;
  while (oi_mode != CREATE_MODE_FULL && oi_mode!=CREATE_MODE_PASSIVE) {
    //while (true)
    //if (serial_port->read(&oi_mode)) break;
    //WaitRead();
    SRead(&oi_mode);
  }
  // read data
  for (unsigned i = 0; i < packets.size(); ++i) {
    for (int j = 0; j < packets[i].numBytes; ++j) {
      //while (true)
      //if (serial_port->read(&packets[i].data[j])) break;
      //WaitRead();
      SRead(&packets[i].data[j]);
    }
  }
  //cout <<endl;
}


int IRobotCreate::toDecimal(uint8_t high, uint8_t low)
{
  //printf("%d %d \n", high, low);
  short dec = high *256 + low;
  short mask = 1 << 15;
  short sign = dec & mask;
  short value = dec & ~mask;
  short sign_value;
  if (sign == mask) sign_value = -(~(value - 1) & ~mask);
    else sign_value = value;
  return sign_value;
}

bool IRobotCreate::setVelocity(double x_vel, double w_vel)
{
//convert linear (m/s) and angular velocity (rad/s) into velocity (mm/s) and radius (mm)
  /*
	int velocity = ((int)(x_vel * 1000.0));	
	int radius;
	if (x_vel == 0.0) {
	  // turning in place
	  if (w_vel > 0.0) {
	    // positive = counter-clockwise
	    radius = CREATE_RADIUS_TURN_CCW;
	    velocity = w_vel * 150.0;
	  } else {                      
	    // negative = clockwise
	    radius = CREATE_RADIUS_TURN_CW;
	    velocity = -w_vel * 150.0;
	  }
	} else if (w_vel == 0.0) {  
	  // straight line
	  radius = CREATE_RADIUS_MAX_MM;
	} else {
	  radius = ((int)(x_vel / w_vel * 1000.0));
	  if (radius > CREATE_RADIUS_MAX_MM) 
	    radius = CREATE_RADIUS_MAX_MM;
	  else if (radius <-CREATE_RADIUS_MAX_MM) 
	    radius = -CREATE_RADIUS_MAX_MM;
	}

	if (velocity > CREATE_TVEL_MAX_MM_S) 
	  velocity = CREATE_TVEL_MAX_MM_S;
	else if (velocity <-CREATE_TVEL_MAX_MM_S) 
	  velocity = -CREATE_TVEL_MAX_MM_S;

	int high1, low1, high2, low2;
	toOpcode(velocity, high1, low1);
	toOpcode(radius, high2, low2);
	
	
	serial_port->write((uint8_t)CREATE_OPCODE_DRIVE);
	Wait();
	serial_port->write((uint8_t)high1);
	Wait();
	serial_port->write((uint8_t)low1);
	Wait();
	serial_port->write((uint8_t)high2);
	Wait();
	serial_port->write((uint8_t)low2);
	Wait();
  */
  double vl = x_vel - w_vel * CREATE_RADIUS;
  double vr = x_vel + w_vel * CREATE_RADIUS;
  IRobotCreate::setWheels(vl, vr);
  //ROS_DEBUG("setting velocity to %d mm/s, radius %d mm \n", velocity, radius);
  //ROS_DEBUG("setting velocity to code [%d][%d], [%d][%d] \n", high1, low1, high2, low2);
  return true;
}

bool IRobotCreate::setWheels(double l_vel, double r_vel)
{
  //left and right wheel velocities (in mm/s)
	int left = l_vel *1000; int right = r_vel*1000;
	//to preserve radius, keep the ratio of left:right the same
	if (left > CREATE_TVEL_MAX_MM_S) { right = right * CREATE_TVEL_MAX_MM_S / left; left= CREATE_TVEL_MAX_MM_S; }
	else if (left < -CREATE_TVEL_MAX_MM_S) { right = - right * CREATE_TVEL_MAX_MM_S / left; left= -CREATE_TVEL_MAX_MM_S; }
	if (right > CREATE_TVEL_MAX_MM_S) {left = left * CREATE_TVEL_MAX_MM_S / right; right= CREATE_TVEL_MAX_MM_S;}
	else if (right < -CREATE_TVEL_MAX_MM_S) {left = -left * CREATE_TVEL_MAX_MM_S / right; right= -CREATE_TVEL_MAX_MM_S;}
	
	int high1, low1, high2, low2;
	toOpcode(right, high1, low1);
	toOpcode(left, high2, low2);
	
	/*
	serial_port->write((uint8_t)CREATE_OPCODE_DRIVE_WHEELS);
	WaitWrite();
	serial_port->write((uint8_t)high1);
	WaitWrite();
	serial_port->write((uint8_t)low1);
	WaitWrite();
	serial_port->write((uint8_t)high2);
	WaitWrite();
	serial_port->write((uint8_t)low2);
	WaitWrite();
	*/

	SWrite((uint8_t)CREATE_OPCODE_DRIVE_WHEELS);
	SWrite((uint8_t)high1);
	SWrite((uint8_t)low1);
	SWrite((uint8_t)high2);
	SWrite((uint8_t)low2);

	//printf("setting velocity to left %d mm/s, right %d mm/s \n", left, right);
	//printf("setting velocity to code [%d][%d], [%d][%d] \n", high1, low1, high2, low2);
	return true;
}

void IRobotCreate::toOpcode(int decimal, int & high, int & low) {
	char hex[16];
	sprintf( hex, "%08x", decimal);
	int first, second;
	if (hex[4]>='0' && hex[4]<='9')	{ first = hex[4] - '0'; }
		else if (hex[4]>='a' && hex[4]<='f')  { first = hex[4] - 'a' +10; }
	if (hex[5]>='0' && hex[5]<='9')	{ second = hex[5] - '0'; }
		else if (hex[5]>='a' && hex[5]<='f')  { second = hex[5] - 'a' +10; }
	high = first *16 + second;
	if (hex[6]>='0' && hex[6]<='9')	{ first = hex[6] - '0'; }
		else if (hex[6]>='a' && hex[6]<='f')  { first = hex[6] - 'a' +10; }
	if (hex[7]>='0' && hex[7]<='9')	{ second = hex[7] - '0'; }
		else if (hex[7]>='a' && hex[7]<='f')  { second = hex[7] - 'a' +10; }
	low = first *16 + second;
}
