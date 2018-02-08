#include <iostream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>


#include <cmath>


#include <pcd_processor/classify.h>
#include <pcd_cloud_saver/save.h>
#include <bayesian_updating/update.h>

#define NODE_NAME "manipulation_node"

#define CLOUD_SAVER_SERVICE_NAME "pcd_cloud_saver_service"
#define CLOUD_PROCESSING_SERVICE_NAME "pcd_processor_service"
#define BAYESIAN_UPDATER "bay_service"


using namespace std;


/* Second scan side
 * 0.606059 Y :0.273793Z :0.114325
 * 0.110882 Y :0.706801 Z :-0.087615 W:0.693153
 *
 * Inter scan
 * :0.697266 Y :0.0817514Z :0.292363
 * :0.138728 Y :0.946421 Z :-0.0266259 W:0.290402
 *
 * Up scan
 * 0.690094 Y :0.0802992Z :0.405642
 * -0.0566094 Y :0.997874 Z :0.00482567 W:0.0319447
 *
 */


geometry_msgs::Pose scan_side_right()
{
	geometry_msgs::Pose pose;

	pose.position.x= 0.606059;
	pose.position.y= 0.273793;
	pose.position.z= 0.114325;

	pose.orientation.x=0.110882;
	pose.orientation.y=0.706801;
	pose.orientation.z=-0.087615;
	pose.orientation.w=0.693153;
 
	return pose;
}


geometry_msgs::Pose scan_inter_right()
{
	geometry_msgs::Pose pose;
	pose.position.x=0.697266;
	pose.position.y=0.0817514;
	pose.position.z=0.292363;

	pose.orientation.x=0.138728;
	pose.orientation.y=0.946421;
	pose.orientation.z=-0.0266259;
	pose.orientation.w=0.290402;

	return pose;
}

geometry_msgs::Pose scan_top()
{
	geometry_msgs::Pose pose;
	
	pose.position.x=0.690094;
	pose.position.y=0.0802992;
	pose.position.z=0.405642;

	pose.orientation.x=-0.0566094;
	pose.orientation.y=0.997874;
	pose.orientation.z=0.00482567;
	pose.orientation.w=0.0319447;
 

	return pose;
}




geometry_msgs::Pose go_scan()
{
	geometry_msgs::Pose pose;

	pose.position.x=0.579694;
	pose.position.y=0.183309;
	pose.position.z=0.113532;


	pose.orientation.x=0.110954;
	pose.orientation.y=0.706495;
	pose.orientation.z=-0.0874397;
	pose.orientation.w=0.693476;



	return pose;

}


void save_cloud()
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<pcd_cloud_saver::save>(CLOUD_SAVER_SERVICE_NAME);

	pcd_cloud_saver::save srv;
	client.call(srv);
}


int process_cloud()
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<pcd_processor::classify>(CLOUD_PROCESSING_SERVICE_NAME);

	pcd_processor::classify srv;

	if(client.call(srv))
	{

	}



	return srv.response.type.data;
}

void update_belief(string viewpoint,int observed_type)
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<bayesian_updating::update>(BAYESIAN_UPDATER);

	bayesian_updating::update srv;
	srv.request.action.data=1;
	srv.request.viewpoint.data=viewpoint;
	srv.request.type.data=observed_type;

	if(client.call(srv))
	{

	}

}

void get_belief(string viewpoint,int observed_type)
{
	ros::NodeHandle nh;
	ros::ServiceClient client=nh.serviceClient<bayesian_updating::update>(BAYESIAN_UPDATER);

	bayesian_updating::update srv;
	srv.request.action.data=2;
	

	if(client.call(srv))
	{

	}

	vector<float> belief=srv.response.belief;

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

	geometry_msgs::Pose pose;
	group_left.setPoseTarget(scan_side_right());
	execute_move(group_left);

	save_cloud();

	int type=process_cloud();

	cout<<"Observed type :"<<type<<endl;


	
	sleep(2.0);
	cout<<"Job finished! Cheers ! :))"<<endl;

	ros::shutdown();


	return 0;
}