#include <iostream>
#include <ros/ros.h>
#include <baxter_gripper_srv/gripper_srv.h>
#include <baxter_current_pose/position.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Vector3.h>


#include <pcd_processor/process.h>
#include <pcd_saver/save.h>

#define NODE_NAME "manipulation_node"

using namespace std;


#define MARKER_SERVICE_NAME "position_service"
#define GRIPPER_SERVICE_NAME "gripper_service"
#define CLOUD_SAVER_SERVICE_NAME "pcd_cloud_saver_service"
#define CLOUD_PROCESSING_SERVICE_NAME "pcd_processor_service"

/*******************************************************
 * Left hand home state                                *
 * 	X :0.579772 Y:0.191197 Z :0.0993143                * 
 *	Orientation --->                                   *
 *	X :0.136569 Y :0.990184 Z :0.0176215 W :0.0239423  *
 * =================================================   *
 * Right hand home state                               *
 * X :0.580194 Y:-0.192612 Z :0.0980529                *
 *	Orientation --->                                   *
 *	X :-0.135652 Y :0.99039 Z :-0.0158881 W :0.0217533 *
 *******************************************************/




void save_cloud()
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<pcd_saver::save>(CLOUD_SAVER_SERVICE_NAME);

	pcd_saver::save srv;
	client.call(srv);
}

geometry_msgs::Vector3 process_cloud()
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<pcd_processor::process>(CLOUD_PROCESSING_SERVICE_NAME);

	pcd_processor::process srv;

	if(client.call(srv))
	{

	}

	geometry_msgs::Vector3 centroid=srv.response.centroid;
	return centroid;

}


void gripper_action(string arm, int action)
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<baxter_gripper_srv::gripper_srv>(GRIPPER_SERVICE_NAME);

	baxter_gripper_srv::gripper_srv srv;

	srv.request.gripper.data=arm;
	srv.request.action.data=action;

	if(client.call(srv))
	{
		cout<<"Success!"<<endl;
	}else
	{
		cout<<"Not success!"<<endl;
	}
}

geometry_msgs::Pose go_scan()
{
	geometry_msgs::Pose pose;

	pose.position.x=0.694775;
	pose.position.y=0.426733;
	pose.position.z=0.321742;

	pose.orientation.x=0.596188;
	pose.orientation.y=0.610899;
	pose.orientation.z=-0.340443;
	pose.orientation.w=0.394286;

	return pose;

}



geometry_msgs::Pose go_above_cube()
{

	geometry_msgs::Pose pose;
	pose.position.x=0.732617;
	pose.position.y=0.134935;
	pose.position.z=0.111202;

	pose.orientation.x=0.231323;
	pose.orientation.y=0.972512;
	pose.orientation.z=0.0197414;
	pose.orientation.w=0.017899;

	return pose;

}



void go_up(moveit::planning_interface::MoveGroup &group)
{
	geometry_msgs::Pose current_pose=group.getCurrentPose().pose;
	current_pose.position.z+=0.1;
	group.setPoseTarget(current_pose);
}



geometry_msgs::Pose get_current_pose(moveit::planning_interface::MoveGroup &group)
{
	group.setStartState(*group.getCurrentState());
	return group.getCurrentPose().pose;
}

void execute_move(moveit::planning_interface::MoveGroup &group,bool special=false)
{

	//group.setGoalPositionTolerance(1e-2);
	if(!special)
	{
		group.setGoalPositionTolerance(0.005);
		group.setMaxVelocityScalingFactor(0.5);

	}else
	{
		group.setGoalPositionTolerance(0.001);
		group.setMaxVelocityScalingFactor(0.25);

	}
	group.setGoalOrientationTolerance(1e-3);
	group.setStartState(*group.getCurrentState());
	
	//group.setMaxAccelerationScalingFactor(0.5);

	moveit::planning_interface::MoveGroup::Plan plan;

	if(group.plan(plan))
	{
		cout<<"Found a plan moving the arm!"<<endl;
		sleep(1.0);
		group.move();
	}else
	{
		cout<<"Plan has not been found!"<<endl;
	}
	

}

geometry_msgs::Pose bulb_tester_position_reconstruct(ros::NodeHandle &nh)
{
	double p_x,p_y,p_z,o_x,o_y,o_z,o_w;

	geometry_msgs::Pose pose;

	nh.getParam("/hole_p_x",p_x);
	nh.getParam("/hole_p_y",p_y);
	nh.getParam("/hole_p_z",p_z);

	nh.getParam("/hole_o_x",o_x);
	nh.getParam("/hole_o_x",o_y);
	nh.getParam("/hole_o_x",o_z);
	nh.getParam("/hole_o_x",o_w);

	pose.position.x=p_x;
	pose.position.y=p_y;
	pose.position.z=p_z;

	pose.orientation.x=o_x;
	pose.orientation.y=o_y;
	pose.orientation.z=o_z;
	pose.orientation.w=o_w;

	return pose;
}


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;


	ros::AsyncSpinner spinner(5);
	spinner.start();

	moveit::planning_interface::MoveGroup group_right("right_arm");
	moveit::planning_interface::MoveGroup group_left("left_arm");

	moveit::planning_interface::MoveGroup::Plan plan_right,plan_left;

	gripper_action("left",1);

	geometry_msgs::Pose pose;
	group_left.setPoseTarget(go_scan());
	execute_move(group_left);

	save_cloud();
	geometry_msgs::Vector3 centroid=process_cloud();
	pose=group_right.getCurrentPose().pose;
	pose.position.x=centroid.x;
	pose.position.y=centroid.y;
	pose.position.z=centroid.z;

	group_left.setPoseTarget(pose);
	execute_move(group_left);

	gripper_action("left",0);

	/*pose=group_right.getCurrentPose().pose;
	cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<"Z :"<<pose.position.z<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W:"<<pose.orientation.w<<endl;
	pose=group_left.getCurrentPose().pose;
	cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<"Z :"<<pose.position.z<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W:"<<pose.orientation.w<<endl;*/


	sleep(2.0);
	cout<<"Job finished!"<<endl;

	ros::shutdown();


	return 0;
}