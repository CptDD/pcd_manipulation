#include <iostream>
#include <ros/ros.h>
#include <baxter_gripper_srv/gripper_srv.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

using namespace std;

#define NODE_NAME "gripper_server"
#define SERVICE_NAME "gripper_service"


bool gripper_service(baxter_gripper_srv::gripper_srv::Request &req,baxter_gripper_srv::gripper_srv::Response &res)
{
	string gripper=req.gripper.data;
	int action=req.action.data;

	ros::NodeHandle nh;

	stringstream ss,args;
	ss<<"/robot/end_effector/"<<gripper<<"_gripper/command";

	ros::Publisher pub=nh.advertise<baxter_core_msgs::EndEffectorCommand>(ss.str(),1);
	baxter_core_msgs::EndEffectorCommand msg;
	
	msg.id=65538;
	msg.command=msg.CMD_GO;	

	if(action==1)
	{
		args<<"{\"position\":100, \"dead zone\":5.0, \"force\":40.0,\"holding force\":30.0,\"velocity\":50.0}";	
	}else
	{
		args<<"{\"position\":0.0, \"dead zone\":5.0, \"force\":50.0,\"holding force\":30.0,\"velocity\":50.0}";
	}


	cout<<ss.str()<<endl;
	cout<<args.str()<<endl;

	msg.args=args.str();

	for(int i=0;i<3;i++)
	{
		pub.publish(msg);
		sleep(1.0);
	}

	return true;


	//ros::Publisher pub=

}


int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;
	ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,gripper_service);

	ROS_INFO("Gripper service up . . .");
	ros::spin();

	return 0;
}