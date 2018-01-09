#include <iostream>
#include <ros/ros.h>
#include <baxter_gripper_srv/gripper_srv.h>

using namespace std;

#define NODE_NAME "gripper_client"
#define SERVICE_NAME "gripper_service"

int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;

	ros::ServiceClient client=nh.serviceClient<baxter_gripper_srv::gripper_srv>(SERVICE_NAME);

	baxter_gripper_srv::gripper_srv srv;

	srv.request.gripper.data="right";
	srv.request.action.data=1;

	if(client.call(srv))
	{
		cout<<"Success!"<<endl;
	}else
	{
		cout<<"Not success!"<<endl;
	}

	return 0;
}

