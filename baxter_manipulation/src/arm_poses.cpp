#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Pose.h>


using namespace std;


int main(int argc,char**argv)
{
	ros::init(argc,argv,"arm_poses_node");
	ros::NodeHandle nh;


	ros::AsyncSpinner spinner(5);
	spinner.start();

	moveit::planning_interface::MoveGroup group_right("right_arm");
	moveit::planning_interface::MoveGroup group_left("left_arm");

	moveit::planning_interface::MoveGroup::Plan plan_right,plan_left;

	geometry_msgs::Pose pose;

	pose=group_right.getCurrentPose().pose;
	cout<<"===RIGHT==="<<endl;
	cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<"Z :"<<pose.position.z<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W:"<<pose.orientation.w<<endl;
	cout<<"===LEFT==="<<endl;
	pose=group_left.getCurrentPose().pose;
	cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<"Z :"<<pose.position.z<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W:"<<pose.orientation.w<<endl;
	cout<<"==========="<<endl;


	sleep(2.0);
	cout<<"Job finished!"<<endl;

	ros::shutdown();


	return 0;
}