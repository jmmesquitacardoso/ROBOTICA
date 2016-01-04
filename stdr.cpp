#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <limits.h>
#include <vector>
#include <stdint.h>

using namespace std;

#define PI 3.14159265359
#define SENSOR_RANGE 300

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Subscriber posSub;
ros::Subscriber occupancySub;
ros::Publisher pub;

float robotRotation = 0.0;
bool rotating = false;
int currentRobotX = 0;
int currentRobotY = 0;

bool followingObstacle = false;


void posMsgReceived(const nav_msgs::Odometry::ConstPtr& msg){
	geometry_msgs::PoseWithCovariance pose = msg->pose;
	currentRobotX = pose.pose.position.x / 0.02;
	currentRobotY = pose.pose.position.y / 0.02;

	if(msg->twist.twist.linear.x == 0 && msg->twist.twist.linear.y == 0){
		followingObstacle = false;
	}
}

void occupancyMsgReceived(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;
	int width = info.width;
	int height = info.height;
	vector< vector<int8_t> > matrix;
	vector<int8_t> line;
	geometry_msgs::Twist msg1;

	int minorDistance = INT_MAX;
	int goalX, goalY;

	ROS_INFO("SIZE = %d %d", height, width);

	

	if (!followingObstacle) {

		for(int i = 0; i < (height * width); i++){
			line.push_back(msg->data[i]);

			if((i + 1) % width == 0){
				matrix.push_back(line);
				line.clear();
			}
		}
		// rotate 360 degrees to start the mapping process
		/*if (robotRotation < PI*2) {
			msg1.angular.z = PI/16;
			robotRotation += PI/16;
			ROS_INFO("Rotation = %f", robotRotation);
		}*/

		for(int i = 0; i < matrix.size(); i++){
			for(int j = 0; j < matrix[i].size(); j++){
				float distance = sqrt(pow((i + currentRobotY),2) + pow((j + currentRobotX),2));
				if(distance >= SENSOR_RANGE && distance <= minorDistance && matrix[i][j] == -1) {
					minorDistance = distance;
					goalX = j;
					goalY = i;
					ROS_INFO("NEXT POINT: %d %d", i, j);
				}
			}
		}

		followingObstacle = true;

		MoveBaseClient ac("move_base", true);

		while(!ac.waitForServer(ros::Duration(5.0))){
   	    	ROS_INFO("Waiting for the move_base action server to come up");
  	   	}

		ROS_INFO("MINOR DISTANCE AT POINT %d %d", goalX, goalY);
		move_base_msgs::MoveBaseGoal goal;
		goal.target_pose.header.frame_id = "robot0";
		goal.target_pose.header.stamp = ros::Time::now();
		goal.target_pose.pose.position.x = goalX*0.02;
		goal.target_pose.pose.position.y = goalY*0.02; 
		goal.target_pose.pose.orientation.w = 1.0;
		
		ac.sendGoal(goal);

		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("GOAL SENT");
		}else{
			ROS_INFO("DEU MERDA");
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "stdr");
	ros::NodeHandle nh;

	srand(time(0));

	occupancySub = nh.subscribe("/gmapping/map",1,&occupancyMsgReceived);
	posSub = nh.subscribe("robot0/odom",1,&posMsgReceived);
	pub = nh.advertise<move_base_msgs::MoveBaseGoal>("move_base_msgs/MoveBaseActionGoal", 1000);

	ros::spin();
}