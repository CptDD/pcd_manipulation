#include <iostream>
#include <ros/ros.h>
#include <pcd_saver/save.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/PointCloud2.h>


#define SERVICE_NAME "pcd_cloud_saver_service"
#define TOPIC_NAME "/camera/depth/points"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void callback(const sensor_msgs::PointCloud2 &msg)
{	
	cloud->clear();
	pcl::fromROSMsg(msg,*cloud);

}


void save_cloud()
{
	pcl::PCDWriter writer;

	cout<<"Saving the cloud!"<<endl;

	const string pkg="pcd_cloud_saver";
	string path = ros::package::getPath(pkg);
  	
	//cout<<"Path :"<<path<<endl;

	stringstream ss;
	ss<<path<<"/clouds/cloud.pcd";

	writer.write(ss.str(),*cloud);
	/*ros::NodeHandle nh;

  	ros::Publisher pub;
  	pub=nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);

           	sensor_msgs::PointCloud2 msg;
           	msg.header.frame_id="virtual_camera_gazebo_optical_frame";
            msg.header.stamp = ros::Time::now();
            pcl::toROSMsg(*cloud,msg);
         


            while(ros::ok())
            {
          	 msg.header.stamp = ros::Time::now();
             pub.publish (msg);
             ros::spinOnce ();
            	   pub.publish(msg);
            }*/

}


bool saver_service(pcd_saver::save::Request &req,pcd_saver::save::Response &res)
{
	cout<<"A request has been made!"<<endl;

	ros::NodeHandle nh;


	ros::Subscriber sub=nh.subscribe(TOPIC_NAME,100,callback);

	do
	{	
		ros::spinOnce();

	}while(cloud->empty());

	cout<<"The cloud has :"<<cloud->points.size()<<endl;

	save_cloud();


	return true;
};


int main(int argc,char**argv)
{
	ros::init(argc,argv,"pcd_cloud_saver");

	ros::NodeHandle nh;

	ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,saver_service);

	ROS_INFO("Point Cloud saving service up . . .");
	ros::spin();

	return 0;
}