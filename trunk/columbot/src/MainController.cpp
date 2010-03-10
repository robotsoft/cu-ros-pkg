/*
 * MainController.cpp
 *
 *  Created on: Jan 29, 2010
 *      Author: joseph
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <staubli/command.h>


#define PI 3.14
#define DEGREE_2_RADIAN(X) (X*PI/180)
#define RAIDAN_2_DEGREE(X) (X*180/PI)

//Test for get current Cartesian position
int testCurrentPosition(ros::ServiceClient client, staubli::command staubli_srv)
{
  staubli_srv.request.command_number = 1;
  if (client.call(staubli_srv))
  {
	ROS_INFO("Staubli Position x : %f [m]", staubli_srv.response.x);
	ROS_INFO("Staubli Position y : %f [m]", staubli_srv.response.y);
	ROS_INFO("Staubli Position z : %f [m]", staubli_srv.response.z);
	ROS_INFO("Staubli Position theta_x : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_x));
	ROS_INFO("Staubli Position theta_y : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_y));
	ROS_INFO("Staubli Position theta_z : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_z));
	return 0;
  }
  else
  {
	ROS_ERROR("Failed to call service staubli");
	return -1;
  }
}

//Test for get current joints
int testGetCurrentJoint(ros::ServiceClient client, staubli::command staubli_srv)
{
  staubli_srv.request.command_number = 2;
  if (client.call(staubli_srv))
  {
	ROS_INFO("Staubli joint 1 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint1));
	ROS_INFO("Staubli joint 2 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint2));
	ROS_INFO("Staubli joint 3 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint3));
	ROS_INFO("Staubli joint 4 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint4));
	ROS_INFO("Staubli joint 5 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint5));
	ROS_INFO("Staubli joint 6 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint6));
	return 0;
  }
  else
  {
	ROS_ERROR("Failed to call service staubli");
	return -1;
  }
}

//test for move joints
int testMoveJoint(ros::ServiceClient client, staubli::command staubli_srv)
{
  staubli_srv.request.command_number = 3;
  staubli_srv.request.joint1 = DEGREE_2_RADIAN(54.18);
  staubli_srv.request.joint2 = DEGREE_2_RADIAN(-30.31);
  staubli_srv.request.joint3 = DEGREE_2_RADIAN(56.6);
  staubli_srv.request.joint4 = DEGREE_2_RADIAN(-8.81);
  staubli_srv.request.joint5 = DEGREE_2_RADIAN(-23.6);
  staubli_srv.request.joint6 = DEGREE_2_RADIAN(-62.0);

  if (client.call(staubli_srv))
  {
	ROS_INFO("Staubli joint 1 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint1));
	ROS_INFO("Staubli joint 2 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint2));
	ROS_INFO("Staubli joint 3 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint3));
	ROS_INFO("Staubli joint 4 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint4));
	ROS_INFO("Staubli joint 5 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint5));
	ROS_INFO("Staubli joint 6 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint6));
	return 0;
  }
  else
  {
	ROS_ERROR("Failed to call service staubli");
	return -1;
  }
}

//Test for move line
int testMoveLine(ros::ServiceClient client, staubli::command staubli_srv)
{
  staubli_srv.request.command_number = 4;
  staubli_srv.request.x = 0.09929; //unit : meter
  staubli_srv.request.y = 0.54831;
  staubli_srv.request.z = 0.59587;
  staubli_srv.request.theta_x = DEGREE_2_RADIAN(-70.11);
  staubli_srv.request.theta_y = DEGREE_2_RADIAN(14.59);
  staubli_srv.request.theta_z = DEGREE_2_RADIAN(73.62);

  if (client.call(staubli_srv))
  {
	  ROS_INFO("Staubli Position x : %f [m]", staubli_srv.response.x);
	  ROS_INFO("Staubli Position y : %f [m]", staubli_srv.response.y);
	  ROS_INFO("Staubli Position z : %f [m]", staubli_srv.response.z);
	  ROS_INFO("Staubli Position theta_x : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_x));
	  ROS_INFO("Staubli Position theta_y : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_y));
	  ROS_INFO("Staubli Position theta_z : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.theta_z));
	  return 0;
  }
  else
  {
	ROS_ERROR("Failed to call service staubli");
	return -1;
  }
}

//Test Forward Kinematics
int testForwardKinematics(ros::ServiceClient client, staubli::command staubli_srv)
{
  staubli_srv.request.command_number = 5;
  staubli_srv.request.joint1 = DEGREE_2_RADIAN(54.18);
  staubli_srv.request.joint2 = DEGREE_2_RADIAN(-30.31);
  staubli_srv.request.joint3 = DEGREE_2_RADIAN(56.6);
  staubli_srv.request.joint4 = DEGREE_2_RADIAN(-8.81);
  staubli_srv.request.joint5 = DEGREE_2_RADIAN(-23.6);
  staubli_srv.request.joint6 = DEGREE_2_RADIAN(-62.0);

  if (client.call(staubli_srv))
  {
	  ROS_INFO("Staubli Position x : %f [m]", staubli_srv.response.x);
	  ROS_INFO("Staubli Position y : %f [m]", staubli_srv.response.y);
	  ROS_INFO("Staubli Position z : %f [m]", staubli_srv.response.z);
	  ROS_INFO("Staubli Position theta_x : %f [degree]", staubli_srv.response.theta_x);
	  ROS_INFO("Staubli Position theta_y : %f [degree]", staubli_srv.response.theta_y);
	  ROS_INFO("Staubli Position theta_z : %f [degree]", staubli_srv.response.theta_z);
	  return 0;
  }
  else
  {
	  ROS_ERROR("Failed to call service staubli");
	  return -1;
  }
}

//Test Forward Kinematics
int testInverseKinematics(ros::ServiceClient client, staubli::command staubli_srv)
{
  staubli_srv.request.command_number = 6;
  staubli_srv.request.x = 0.53534; //unit : meter
  staubli_srv.request.y = 0.12191;
  staubli_srv.request.z = 0.550;
  staubli_srv.request.theta_x = DEGREE_2_RADIAN(-44.76);
  staubli_srv.request.theta_y = DEGREE_2_RADIAN(67.37);
  staubli_srv.request.theta_z = DEGREE_2_RADIAN(58.06);

  if (client.call(staubli_srv))
  {
	ROS_INFO("Staubli joint 1 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint1));
	ROS_INFO("Staubli joint 2 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint2));
	ROS_INFO("Staubli joint 3 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint3));
	ROS_INFO("Staubli joint 4 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint4));
	ROS_INFO("Staubli joint 5 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint5));
	ROS_INFO("Staubli joint 6 : %f [degree]", RAIDAN_2_DEGREE(staubli_srv.response.joint6));
	  return 0;
  }
  else
  {
	  ROS_ERROR("Failed to call service staubli");
	  return -1;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MainController");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<staubli::command>("command");
  staubli::command staubli_srv;

  //Test command service of staubli package
  testCurrentPosition(client, staubli_srv);
  testGetCurrentJoint(client, staubli_srv);
  testMoveJoint(client, staubli_srv);
  testMoveLine(client, staubli_srv);
  testForwardKinematics(client, staubli_srv);
  testInverseKinematics(client, staubli_srv);

  return 0;
}
