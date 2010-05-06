#include "irobot_create/irobot_create.h"

int main(int, char **)
{
  IRobotCreate robot;
  robot.Start();

  double x =0,y=0,th=0;
  double x_vel, w_vel;
  time_t sec;
  
  x_vel = 0.2; w_vel = 0.0;
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;
  robot.setVelocity(x_vel,w_vel);
  sec = time(NULL);
  while (time(NULL) - sec < 4.0) {
  }
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;

  x_vel = -0.2; w_vel = 0.0;
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;
  robot.setVelocity(x_vel,w_vel);
  sec = time(NULL);
  while (time(NULL) - sec < 4.0) {
  }
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;

  x_vel = 0.2; w_vel = 0.4;
  robot.setVelocity(x_vel,w_vel);
  sec = time(NULL);
  while (time(NULL) - sec < 3.14159/ w_vel - 0.1) {
    robot.updateStates();
    robot.getOdometricPos(x,y,th);
    cout <<"x= " << x << " y= "<< y << " th =" << th << endl;
    usleep(100000);
  }  

  x_vel = 0.20; w_vel = 0.0;
  robot.setVelocity(x_vel,w_vel);
  sec = time(NULL);
  while (time(NULL) - sec < 4.0) {
  }
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;

  x_vel = 0.0; w_vel = -0.3;
  robot.setVelocity(x_vel,w_vel);
  sec = time(NULL);
  while (time(NULL) - sec < -1.5* 3.14159/ w_vel) {
	}
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;  
    
  robot.setVelocity(0.0, 0.0);
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;  

  return 0;
}

