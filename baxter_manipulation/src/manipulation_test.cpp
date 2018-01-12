#include <iostream>
#include <ros/ros.h>
#include <baxter_gripper_srv/gripper_srv.h>
#include <moveit/move_group_interface/move_group.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>


#include <pcd_processor/process.h>
#include <pcd_cloud_saver/save.h>

#define NODE_NAME "manipulation_node"

using namespace std;


#define MARKER_SERVICE_NAME "position_service"
#define GRIPPER_SERVICE_NAME "gripper_service"
#define CLOUD_SAVER_SERVICE_NAME "pcd_cloud_saver_service"
#define CLOUD_PROCESSING_SERVICE_NAME "pcd_processor_service"



void save_cloud()
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<pcd_saver::save>(CLOUD_SAVER_SERVICE_NAME);

	pcd_cloud_saver::save srv;
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


geometry_msgs::Pose pre_pick()
{
	
	geometry_msgs::Pose pose;


	pose.position.x=0.684235;
	pose.position.y=0.272976;
	pose.position.z=0.27912;

	pose.orientation.x=0.483781;
	pose.orientation.y=0.507936;
	pose.orientation.z=-0.487413;
	pose.orientation.w=0.519986;

	return pose;
}


void go_adv(moveit::planning_interface::MoveGroup &group)
{
	geometry_msgs::Pose current_pose=group.getCurrentPose().pose;
	current_pose.position.x+=0.03;
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

	cout<<"Executing orientation"<<endl;

	group_left.setPoseTarget(pre_pick());
	execute_move(group_left);

	pose=group_left.getCurrentPose().pose;
	pose.position.y=centroid.y+0.03;
	pose.position.x=0.73;

	group_left.setPoseTarget(pose);
	execute_move(group_left);

	pose=group_left.getCurrentPose().pose;
	pose.position.y=centroid.y;

	group_left.setPoseTarget(pose);
	execute_move(group_left);

	
	pose=group_left.getCurrentPose().pose;
	pose.position.z=floor(centroid.y);

	group_left.setPoseTarget(pose);
	execute_move(group_left);


	
	pose=group_left.getCurrentPose().pose;
	pose.position.y=floor(centroid.y)-0.12;


	group_left.setPoseTarget(pose);
	execute_move(group_left);

	gripper_action("left",0);


	pose=group_left.getCurrentPose().pose;
	pose.position.z+=0.2;


	group_left.setPoseTarget(pose);
	execute_move(group_left);

	


	sleep(2.0);
	cout<<"Job finished! Cheers ! :))"<<endl;

	ros::shutdown();


	return 0;
}