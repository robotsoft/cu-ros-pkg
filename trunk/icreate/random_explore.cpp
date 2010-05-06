#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <robot_msgs/PoseStamped.h>
#include <robot_msgs/PoseWithCovariance.h>
#include "irobot_create/irobot_create.h"
#include <stdlib.h>
#include <time.h>
#define NUM_POINTS 6
#define BATT_THRESHOLD_LOW 89
#define BATT_THRESHOLD_HIGH 90

static float location[NUM_POINTS][4] = {{41.5,21.0,0.9788,-0.2047},
			       {26.7,19.25,-0.6653,0.7554},
			       {6.6,16.7,-0.6606,0.7507},
			       {8.1,7.1,-0.0597,0.9982},
			       {27.3,9.5,0.0825,0.9966},
			       {44.5,11.7,0.9931,0.1175}};
static float connection[NUM_POINTS][NUM_POINTS] = {{0,1,0,0,0,0},{1,0,1,0,0,0},{0,1,0,1,0,0},{0,0,1,0,1,0},{0,0,0,1,0,1},{0,0,0,0,1,0}};
static bool ready = true;
static bool docking = false;
static bool robot_startdockauto = false;
static bool robot_startmoving = false;
static robot_msgs::PoseStamped goal;
static int goal_id = 1;

void receiveGoal(const boost::shared_ptr<const robot_msgs::PoseStamped> goal)
{
  robot_msgs::Point position = goal->pose.position;
  robot_msgs::Quaternion orientation = goal->pose.orientation;
  cout << "###received goal (position): " << position.x << ' ' << position.y << ' ' << position.z << endl;
  cout << "###received goal (orientation): " << orientation.x << ' ' << orientation.y << ' ' << orientation.z << ' ' << orientation.w <<endl;
  //cout << "convert to angle" << 2 * acos(orientation.w) << endl;
}

void receiveAmcl(const boost::shared_ptr<const robot_msgs::PoseWithCovariance> amcl_pose)
{
  robot_msgs::Point position = amcl_pose->pose.position;
  robot_msgs::Quaternion orientation = amcl_pose->pose.orientation;
  //float angle = 2 * acos(orientation.w);
  
  //cout << "###received robot position: " << position.x << ' ' << position.y << ' ' << position.z << endl;
  //cout << "###received robot orientation: " << orientation.x << ' ' << orientation.y << ' ' << orientation.z << ' ' << orientation.w <<endl;
  //for (int i=0; i<6; i++) 
  if (!robot_startdockauto && sqrt(pow(location[goal_id][0]-position.x,2)+pow(location[goal_id][1]-position.y,2)) < 0.2) {
    //cout << 'z' << orientation.z << " w " << orientation.w <<endl;
    if ( abs(orientation.w-location[goal_id][3]) < 0.5){
      if (!docking) {
	ready = true;
	int next = rand() % NUM_POINTS;
	while (!connection[goal_id][next])
	  next = rand() % NUM_POINTS;
	cout << "going to location #" << next << endl;
	goal.pose.position.x = location[next][0];
	goal.pose.position.y = location[next][1];
	goal.pose.position.z = 0;
	goal.pose.orientation.x = 0;
	goal.pose.orientation.y = 0;
	goal.pose.orientation.z = location[next][2];
	goal.pose.orientation.w = location[next][3];
	goal.header.frame_id = "/map";
	goal_id = next;
      } else {
	robot_startdockauto = true;
      }
    }
  }
}
void receiveBatt(const boost::shared_ptr<const std_msgs::Int32> batt_robot) 
{						
  int batt = batt_robot->data;
  printf("batt: %d\n", batt);
  if (batt < BATT_THRESHOLD_LOW && !docking) {
    goal.pose.position.x = location[1][0];
    goal.pose.position.y = location[1][1];
    goal.pose.position.z = 0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = location[1][2];
    goal.pose.orientation.w = location[1][3];
    goal.header.frame_id = "/map";
    goal_id = 1;
    cout << "go to recharge" << endl;
    docking = true;
    ready = true;
  } else if (batt > BATT_THRESHOLD_HIGH && robot_startdockauto) {
    goal.pose.position.x = location[1][0];
    goal.pose.position.y = location[1][1];
    goal.pose.position.z = 0;
    goal.pose.orientation.x = 0;
    goal.pose.orientation.y = 0;
    goal.pose.orientation.z = location[1][2];
    goal.pose.orientation.w = location[1][3];
    goal.header.frame_id = "/map";
    goal_id = 1;
    robot_startdockauto = false;
    robot_startmoving = true;
    docking = false;
    ready = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "random_explore");
  ros::NodeHandle n;
  //n.advertise<robot_msgs::PoseDot>("cmd_vel",1);
  ros::Subscriber goal_sub = n.subscribe<robot_msgs::PoseStamped>("goal", 2,  receiveGoal);
  ros::Subscriber amcl_sub = n.subscribe<robot_msgs::PoseWithCovariance>("amcl_pose", 2,  receiveAmcl);
  ros::Subscriber batt_sub = n.subscribe<std_msgs::Int32>("batt", 1, receiveBatt);
  ros::Publisher goal_pub = n.advertise<robot_msgs::PoseStamped>("/move_base/activate", 2);
  ros::Publisher autodock_pub = n.advertise<std_msgs::Int32>("autodock", 1);
  ros::Rate loop_rate(10);
  srand(time(NULL));
  cout << "initial location#" << goal_id << endl;
  goal.pose.position.x = location[goal_id][0];
  goal.pose.position.y = location[goal_id][1];
  goal.pose.position.z = 0;
  goal.pose.orientation.x = 0;
  goal.pose.orientation.y = 0;
  goal.pose.orientation.z = location[goal_id][2];
  goal.pose.orientation.w = location[goal_id][3];
  goal.header.frame_id = "/map";
  loop_rate.sleep();
  goal_pub.publish(goal);
  while (n.ok()) {
    ros::spinOnce();
    if (ready) {
      goal_pub.publish(goal); 
      ready = false;
    }
    std_msgs::Int32 indicator;
    if (robot_startdockauto) {
      indicator.data = 1;
      autodock_pub.publish(indicator);
    } else if (robot_startmoving) {
      indicator.data = 0;
      autodock_pub.publish(indicator);
    }
    loop_rate.sleep();
    }
  exit(0);
}
