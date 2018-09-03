#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/tf.h>
//#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>

#include <bulb_scanner/scan.h>
#include <gazebo_msgs/SpawnModel.h>

#define SCAN_SERVICE "/bulb_scan"
#define GAZEBO_SERVICE "/gazebo/spawn_sdf_model"



using namespace std;


void show_poses(vector<geometry_msgs::Pose> &poses)
{
	for(int i=0;i<poses.size();i++)
	{
		cout<<"X :"<<poses[i].position.x<<" Y :"<<poses[i].position.y<<" Z :"<<poses[i].position.z<<endl;
		cout<<"W :"<<poses[i].orientation.w<<" X :"<<poses[i].orientation.x<<" Y :"<<poses[i].orientation.y<<" Z :"<<poses[i].orientation.z<<endl;
	}
}


string read_file(string path)
{
	string line;

  	ifstream file(path.c_str());

  	stringstream ss;


  	if (file.is_open())  		
  	{
  		while(getline(file,line))
  			{
  				ss<<line<<endl;
  				//cout << line << '\n';
  			}
  		file.close();

  	}else
  	{ 
  		cout << "Unable to open file"; 
	}  	

	return ss.str();
}

int main(int argc,char**argv)
{
	ros::init(argc,argv,"test_scan_node");
	ros::NodeHandle nh;

	ros::ServiceClient client=nh.serviceClient<bulb_scanner::scan>(SCAN_SERVICE);

	bulb_scanner::scan srv;
	srv.request.center.position.x=0.6;
	srv.request.center.position.y=0;
	srv.request.center.position.z=0.82;

	srv.request.radius.data=0.3;
	srv.request.nr_of_points.data=10;

	string reachable=read_file("/home/cptd/.gazebo/models/bulb_marker_good/model.sdf");
	string non_reachable=read_file("/home/cptd/.gazebo/models/bulb_marker/model.sdf");

	vector<geometry_msgs::Pose> poses;

	if(client.call(srv))
	{
		cout<<"Success !"<<endl;

		poses=srv.response.points;


		/*

		spawn_client.call(spawn_srv);*/


	}else
	{
		cout<<"Not success !"<<endl;
	}

	cout<<poses.size()<<endl;

	ros::AsyncSpinner spinner(5);
	spinner.start();

	moveit::planning_interface::MoveGroupInterface group_right("right_arm");
	moveit::planning_interface::MoveGroupInterface group_left("left_arm");

	moveit::planning_interface::MoveGroupInterface::Plan plan_right,plan_left;

	geometry_msgs::Pose pose;

	pose=group_right.getCurrentPose().pose;
	cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<"Z :"<<pose.position.z<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W:"<<pose.orientation.w<<endl;
	pose=group_left.getCurrentPose().pose;
	cout<<"X :"<<pose.position.x<<" Y :"<<pose.position.y<<"Z :"<<pose.position.z<<endl;
	cout<<"X :"<<pose.orientation.x<<" Y :"<<pose.orientation.y<<" Z :"<<pose.orientation.z<<" W:"<<pose.orientation.w<<endl;

	ros::ServiceClient spawn_client=nh.serviceClient<gazebo_msgs::SpawnModel>(GAZEBO_SERVICE);


	for(int i=0;i<poses.size();i++)
	{
		gazebo_msgs::SpawnModel spawn_srv;

		geometry_msgs::Pose target=poses[i];

		/*target.position.x=poses[i].position.x;
		target.position.y=poses[i].position.y;
		target.position.z=poses[i].position.z;*/

		group_left.setPoseTarget(target);
		group_left.setGoalOrientationTolerance(1e-3);
		group_left.setStartState(*group_left.getCurrentState());

		stringstream s;
		s<<"bulb_"<<i;

		spawn_srv.request.initial_pose.position.x=poses[i].position.x;
		spawn_srv.request.initial_pose.position.y=poses[i].position.y;
		spawn_srv.request.initial_pose.position.z=poses[i].position.z;
		spawn_srv.request.model_name=s.str();


		if(group_left.plan(plan_left))
		{
			cout<<"Found a plan for pose :"<<i<<endl;
			spawn_srv.request.model_xml=reachable;
			int b;
			cout<<"Enter"<<endl;
			cin>>b;
			group_left.move();
		}else
		{
			cout<<"No plan for pose :"<<i<<endl;
			spawn_srv.request.model_xml=non_reachable;
		}

		spawn_client.call(spawn_srv);
	}

	cout<<"Job finished!"<<endl;
	ros::shutdown();
	return 0;
}