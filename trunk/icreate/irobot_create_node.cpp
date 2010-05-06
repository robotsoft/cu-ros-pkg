#include <iostream>
#include <math.h>
#include "time.h"
#include <assert.h>
//#include <ros/node.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <robot_msgs/Pose.h>
#include <robot_msgs/PoseDot.h>
#include <robot_msgs/PoseWithCovariance.h>
#include <robot_msgs/PoseWithRatesStamped.h>
#include <deprecated_msgs/RobotBase2DOdom.h>
#include <deprecated_msgs/Pose2DFloat32.h>
#include <tf/transform_broadcaster.h>
#include <joy/Joy.h>
#include "irobot_create/irobot_create.h"

extern "C"
{
#include <libacpi.h>
}

//robot_msgs::PoseWithCovariance initialPos; // Initial position message.
double initX, initY, initTh;               // Initial position, simplified.
bool initialized;                          // Whether the initial location has been 
//robot_msgs::PoseDot velCmd;                // Current velocity message.
//deprecated_msgs::RobotBase2DOdom odom;
IRobotCreate robot;                        // Robot we pass the message to.
bool passthrough = true;
bool docking = false;
global_t * globals;
ros::Time relocalize_start;

void initTest()
{
  initX = 23;
  initY = 13;
  initTh = 0;
  initialized = true;
}

void
relocalize()
{
  robot.setVelocity(-0.35,0.0);
  ros::Rate back_rate(1);
  back_rate.sleep();
  robot.setVelocity(0.0, 0.8);
  ros::Rate turn_rate(0.1);
  turn_rate.sleep();
}

void updateBumper() 
{
  int bump = 0;
  robot.getBumper(bump);
  if (bump) {
    passthrough = false;
    relocalize_start = ros::Time::now();
  }
}

void updateBatt(std_msgs::Int32 & batt_robot)
{
  int batt_bot = 100;
  robot.getBatt(batt_bot);
  batt_robot.data =  batt_bot;
  printf("%d \n", batt_bot);
  if (globals->batt_count > 0) {
    read_acpi_batt(0);
    battery_t batt_com = batteries[0];
    printf("%d \n", batt_com.percentage);
  }  
}

void updateStates(deprecated_msgs::RobotBase2DOdom & odom, tf::TransformBroadcaster & odom_msg, std_msgs::Int32 & batt_robot) 
{
  //assert(initialized);
  double dx, dy, dth;
  deprecated_msgs::Pose2DFloat32 pos, vel;
  robot.updateStates();
  robot.getOdometricPos(dx, dy, dth);
  pos.x = dx;
  pos.y = dy;
  pos.th = dth;
  odom.pos = pos;
  //odom.vel = vel;
  cout << "---publish odom " << pos.x << ' ' << pos.y << ' ' << pos.th << endl;
  //tf::TransformBroadcaster tfmsg;
  tf::Quaternion rot(odom.pos.th,0,0);
  //tf::QuaternionTFToMsg(rot, odom.p);
  odom_msg.sendTransform(tf::Stamped<tf::Transform>(tf::Transform(rot,tf::Point(odom.pos.x,odom.pos.y,0.0)),ros::Time::now(), "base_footprint", "odom"));
  if (!docking) updateBumper();
  updateBatt(batt_robot);
}

#if 0
void updateGroundTruth(robot_msgs::PoseWithRatesStamped & odom, tf::TransformBroadcaster & ground_truth_msg, double dx, double dy, double dth)
{
  robot_msgs::Pose pos;//, vel;
  pos.position.x = initX + cos(initTh)*dx - sin(initTh)*dy;
  pos.position.y = initY + sin(initTh)*dx + cos(initTh)*dy;
  pos.position.z = 0;
  pos.orientation.x = pos.orientation.y = 0.0;
  double angle = initTh+dth;
  pos.orientation.z = sin(angle/2.0);
  pos.orientation.w = cos(angle/2.0);
  odom.pos = pos;
  cout << "***publish ground truth " << pos.position.x << ' ' << pos.position.y << ' ' << initTh+dth << endl; 
  /*
  tf::Quaternion rot(angle,0,0);
  ground_truth_msg.sendTransform(tf::Stamped<tf::Transform>(tf::Transform(rot,tf::Point(pos.position.x,pos.position.y,0.0)),ros::Time::now(),"/base_footprint", "/map"));
  */
}
#endif

void receiveVelCmd(const boost::shared_ptr<const robot_msgs::PoseDot> velCmd)
{
  if (!docking) {
    if (passthrough) {
      double xVel = velCmd->vel.vx;
      double wVel = velCmd->ang_vel.vz;
      cout << "###received vel command: " << xVel << ' ' << wVel << endl;
      robot.setVelocity(xVel, wVel);
    } else {
      ros::Duration diff = ros::Time::now() - relocalize_start;
      if (diff.toSec()<1.0) robot.setVelocity(-0.35,0.0);
      else if (diff.toSec()<11.0) robot.setVelocity(0.0, 0.8);
      else passthrough = true;      
    }
  }
}

void receiveSong(const boost::shared_ptr<const joy::Joy> joy_msg)
{
  //for (int i =0; i< 20; i++)
  //cout << " (" << i << ") " << joy_msg->buttons[i];
  //cout << endl;
  if (!docking && joy_msg->buttons[2]) {
    //cout << "hits here!" << endl; 
    robot.playSong(1);
  }
}

void receiveDock(const boost::shared_ptr<const std_msgs::Int32> indicator)
{
  if (indicator->data) {
    docking = true;
    robot.Dock();
  } else {
    docking = false;
    robot.FullControl();
  }
}

#if 0
void
resetPosition(const boost::shared_ptr<const robot_msgs::PoseWithCovariance> initialPos)
{
  tf::TransformBroadcaster posmsg;
  initX = initialPos->pose.position.x;
  initY = initialPos->pose.position.y;
  initTh = 2 * acos(initialPos->pose.orientation.w);

  // flip angle if necessary
  if (initialPos->pose.orientation.z <= 0) 
    {
    if (initTh/2 < M_PI)
      initTh = -initTh;
    } 
  else 
    {
    if (initTh/2 >= M_PI)
      initTh = -initTh;
    }
  double dx, dy, dth;
  robot.getOdometricPos(dx, dy, dth);
  robot.resetDisplacements();
  
  cout << "###position initialized to "<< initX << ' ' << initY << ' ' << initTh << endl;
  initialized = true;
}
#endif

int main(int argc, char ** argv) 
{
  ros::init(argc, argv, "irobot_create");
  ros::NodeHandle n;
  //robot_msgs::PoseWithRatesStamped ground_truth;
  deprecated_msgs::RobotBase2DOdom odom;
  std_msgs::Int32 batt_robot;
  robot.Start();
  globals = (global_t *)malloc(sizeof(global_t));
  init_acpi_batt(globals);
  initialized = false;
  tf::TransformBroadcaster tf;
  ros::Publisher batt_pub = n.advertise<std_msgs::Int32>("/batt",1);
  ros::Publisher odom_pub = n.advertise<deprecated_msgs::RobotBase2DOdom>("/odom", 50); //deprecated_msgs::RobotBase2DOdom
  //ros::Publisher ground_truth_pub = n.advertise<robot_msgs::PoseWithRatesStamped>("/base_pose_ground_truth", 50);
  //ros::Publisher time = n.advertise<roslib::Time>("/time",10);
  //ros::Subscriber init_sub = n.subscribe<robot_msgs::PoseWithCovariance>("/initialpose", 1, resetPosition);
  ros::Subscriber vel_sub = n.subscribe<robot_msgs::PoseDot>("/cmd_vel", 10,  receiveVelCmd);
  ros::Subscriber song_sub = n.subscribe<joy::Joy>("/joy", 1, receiveSong);
  ros::Subscriber dock_sub = n.subscribe<std_msgs::Int32>("/autodock",1, receiveDock);
  ros::Rate loop_rate(10);
  //initTest();
  while (n.ok()) {
    ros::spinOnce();
    //tf.sendTransform(tf::Stamped<tf::Transform>(txIdentity,ros::Time::now(),"base_laser","base_link"));
    //tf.sendTransform(tf::Stamped<tf::Transform>(txIdentity,ros::Time::now(),"base_link","base_footprint"));
    //if (initialized) {
    updateStates(odom, tf, batt_robot);
    odom_pub.publish(odom);
    batt_pub.publish(batt_robot);
      //updateGroundTruth(ground_truth, tf, odom.pos.x, odom.pos.y, odom.pos.th);
      //ground_truth_pub.publish(ground_truth);
      //tf::Quaternion rot(initTh,0,0);
      //tf.sendTransform(tf::Stamped<tf::Transform>(tf::Transform(rot,tf::Point(initX, initY, 0.0)),ros::Time::now(), "/odom", "/map"));
      //}
    loop_rate.sleep();
  }

  cout << "Stopping robot." << endl;
  robot.setVelocity(0, 0);
  
  exit(0);
}
