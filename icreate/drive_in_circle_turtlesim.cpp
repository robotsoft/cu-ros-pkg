#include <ros/ros.h>
#include <turtlesim/Velocity.h>
#include "irobot_create/irobot_create.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv,"create_turtle");
  ros::NodeHandle this_node;
  ros::Publisher vel_pub_ = this_node.advertise<turtlesim::Velocity>("turtle1/command_velocity", 1);

  turtlesim::Velocity vel;
  double l_scale=3, a_scale=1.0;
  
  IRobotCreate robot;
  robot.Start();

  double x =0,y=0,th=0;
  double x_vel, w_vel;
  time_t sec;

  
  x_vel = 0.2; w_vel = 0.0;
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;

  //Update turtlesime
  vel.angular = w_vel*a_scale;
  vel.linear = x_vel*l_scale;
  vel_pub_.publish(vel); 

  robot.setVelocity(x_vel,w_vel);
  sec = time(NULL);
  while (time(NULL) - sec < 4.0) {
	vel_pub_.publish(vel); 
  }
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;

  x_vel = -0.2; w_vel = 0.0;
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;
  robot.setVelocity(x_vel,w_vel);

  //Update turtlesime
  vel.angular = w_vel*a_scale;
  vel.linear = x_vel*l_scale;
  vel_pub_.publish(vel);

  sec = time(NULL);
  while (time(NULL) - sec < 4.0) {
	vel_pub_.publish(vel); 
  }
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;

  x_vel = 0.2; w_vel = 0.4;
  robot.setVelocity(x_vel,w_vel);

  //Update turtlesime
  vel.angular = w_vel*a_scale;
  vel.linear = x_vel*l_scale;
  vel_pub_.publish(vel);

  sec = time(NULL);
  while (time(NULL) - sec < 3.14159/ w_vel - 0.1) {
    robot.updateStates();
    robot.getOdometricPos(x,y,th);
    cout <<"x= " << x << " y= "<< y << " th =" << th << endl;
    vel_pub_.publish(vel);
    usleep(100000);
  }  

  x_vel = 0.20; w_vel = 0.0;
  robot.setVelocity(x_vel,w_vel);
   
  //Update turtlesime
  vel.angular = w_vel*a_scale;
  vel.linear = x_vel*l_scale;
  vel_pub_.publish(vel);

  sec = time(NULL);
  while (time(NULL) - sec < 4.0) {
	vel_pub_.publish(vel); 
  }
  robot.updateStates();
  robot.getOdometricPos(x,y,th);
  cout <<"x= " << x << " y= "<< y << " th =" << th << endl;
  
  x_vel = 0.0; w_vel = -0.3;
  robot.setVelocity(x_vel,w_vel);

  //Update turtlesime
  vel.angular = w_vel*a_scale;
  vel.linear = x_vel*l_scale;
  vel_pub_.publish(vel);

  sec = time(NULL);
  while (time(NULL) - sec < -1.5* 3.14159/ w_vel) {
  vel_pub_.publish(vel);
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

