#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Header.h>
#include <nav_msgs/OccupancyGrid.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <stdint.h>

using namespace std;

#define PI 3.14159265359

ros::Subscriber laserSub;
ros::Subscriber posSub;
ros::Subscriber occupancySub;
ros::Publisher pub;
sensor_msgs::Range rangeMsg;

float robotRotation = 0.0;
bool rotating = false;
int currentRobotX = 0;
int currentRobotY = 0;
bool followingObstacle = false;

void range0MessageReceived (const sensor_msgs::Range& msg) {
	
	rangeMsg = msg;

	ros::Rate rate(1);

	geometry_msgs::Twist msg1;

	ROS_INFO_STREAM("Range = " << rangeMsg.range);
	if (rangeMsg.range <= 1.0 && rotating == false) {
		ROS_INFO_STREAM("ROTATING");
		int direction = rand() % 2;
		if(direction == 0){
			robotRotation = PI/2.0;
		}else{
			robotRotation = -PI/2.0;
		}		
		msg1.angular.z = robotRotation;
		rotating = true;
	} else {
		ROS_INFO("GOING ON");
		rotating = false;
		msg1.linear.x = 1;
	}


	pub.publish(msg1);

	rate.sleep();

}

void posMsgReceived(const nav_msgs::Odometry::ConstPtr& msg){
	/*ROS_INFO("POSITION INFO");
	geometry_msgs::PoseWithCovariance pose = msg->pose;
	ROS_INFO("POSITION = %f %f", pose.pose.position.x, pose.pose.position.y);
	ROS_INFO("POSITION IN PIXELS = %f %f", pose.pose.position.x/0.02, pose.pose.position.y/0.02);*/
}

void occupancyMsgReceived(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	std_msgs::Header header = msg->header;
	nav_msgs::MapMetaData info = msg->info;
	int width = info.width;
	int height = info.height;
	vector< vector<int8_t> > matrix;
	vector<int8_t> line;
	geometry_msgs::Twist msg1;

	ROS_INFO("SIZE = %d %d", height, width);

	for(int i = 0; i < (height * width); i++){
		line.push_back(msg->data[i]);
		
		if((i + 1) % width == 0){
			matrix.push_back(line);
			line.clear();
		}
	}

	if (!followingObstacle) {
		// rotate 360 degrees to start the mapping process
		if (robotRotation < PI*2) {
			msg1.angular.z = PI/16;
			robotRotation += PI/16;
			ROS_INFO("Rotation = %f", robotRotation);
		}
		pub.publish(msg1);
	}
	
	for(int i = 0; i < matrix.size(); i++){
		for(int j = 0; j < matrix[i].size(); j++){
			if(sqrt(pow((i + currentRobotX),2) + pow((j + currentRobotY),2))) {
				// call a star here to get the path to the objective
				followingObstacle = true;
			}
		}
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "stdr");
	ros::NodeHandle nh;

	srand(time(0));

	//laserSub = nh.subscribe("robot0/sonar_0",1,&range0MessageReceived);
	occupancySub = nh.subscribe("/gmapping/map",1,&occupancyMsgReceived);
	posSub = nh.subscribe("robot0/odom",1,&posMsgReceived);
	pub = nh.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);

	ros::spin();
}