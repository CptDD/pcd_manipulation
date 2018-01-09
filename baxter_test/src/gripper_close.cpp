#include <iostream>
#include <ros/ros.h>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/EndEffectorState.h>

using namespace std;

#define NODE_DESCRIPTION "A node for controlling Baxter's grippers"
#define NODE_NAME "baxter_gripper_controller"

bool ok=false;



int main(int argc,char**argv)
{
	ros::init(argc,argv,NODE_NAME);
	ros::NodeHandle nh;


	/*ros::ServiceClient service=nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	gazebo_msgs::GetModelState srv;
	srv.request.model_name="beer_bottle";
	

	if(service.call(srv))
	{
		geometry_msgs::Pose pose=srv.response.pose;
		cout<<pose.position.x<<" "<<pose.position.y<<endl;
	}else
	{
		cout<<"An error has appeared!"<<endl;
	}*/




	ros::Publisher pub_left,pub_right;


	pub_left=nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command",1);	
	pub_right=nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command",1);
	
	
	baxter_core_msgs::EndEffectorCommand msg;
	
	msg.id=65538;
	msg.command=msg.CMD_GO;	
	


	stringstream args;
	args<<"{\"position\":100, \"dead zone\":5.0, \"force\":40.0,\"holding force\":30.0,\"velocity\":50.0}";	
	

	stringstream args2;
	args2<<"{\"position\":0.0, \"dead zone\":5.0, \"force\":40.0,\"holding force\":30.0,\"velocity\":50.0}";	

	msg.args=args2.str();

	
	for(int i=0;i<5;i++)
	{
		pub_left.publish(msg);	
		pub_right.publish(msg);
		sleep(1.0);
	}
	



	/*std::stringstream gripper_command_args;
	gripper_command_args << "{ \"position \": 100.0, \"dead zone\": 5.0, \"force\": 40.0, \"holding force\": 30.0, \"velocity\": 50.0}";
	
	msg.args = gripper_command_args.str().c_str();
	msg.sender="AA";
	msg.sequence=1;

	pub.publish(msg);
	
	sleep(10.0);
	
	*/
	cout<<"Done"<<endl;	
	


	
	return 0;
	
}
