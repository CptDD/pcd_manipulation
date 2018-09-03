#!/usr/bin/python

import rospy
import numpy as np
import math
import copy
import sys

import baxter_interface


from bulb_scanner.srv import *
from pcd_cloud_saver.srv import *

from geometry_msgs.msg import *
from std_msgs.msg import *

from gazebo_msgs.srv import *
from baxter_core_msgs.srv import*
from tf.transformations import quaternion_from_euler



def random_on_sphere_points(r,num):
    points=[]
    for i in range(0,num):
        z=random.uniform(-r,r)
        phi=random.uniform(0,2*math.pi)
        x = math.sqrt(r**2 - z**2)*math.cos(phi)
        y = math.sqrt(r**2 - z**2)*math.sin(phi)
        points.append([x,y,z])
    return points


def regular_on_sphere_points(r,num):
    points=[]

    if num==0:
        return points

    a = 4.0 * math.pi*(r**2.0 / num)

    print(a)

    d = math.sqrt(a)
    m_theta = int(round(math.pi / d))
    d_theta = math.pi / m_theta
    d_phi = a / d_theta

    for m in range(0,m_theta):
        theta = math.pi * (m + 0.5) / m_theta
        m_phi = int(round(2.0 * math.pi * math.sin(theta) / d_phi))
        for n in range(0,m_phi):
            phi = 2.0 * math.pi * n / m_phi
            x = r * math.sin(theta) * math.cos(phi)
            y = r * math.sin(theta) * math.sin(phi)
            z = r * math.cos(theta)
            points.append([x,y,z])
    return points


def only(points,center):
	new_points=[]
	for point in points:
		if point[2]>=center.z:
			new_points.append(point)

	return new_points



def center_points(points,center):
	for point in points:
		point[0]+=center.x
		point[1]+=center.y
		point[2]+=center.z
	return points


def show_points(points):
	for point in points:
		print(point)


def compute_rot_x(point):
    if(point[1]!=0):
        rot_x=math.atan(abs(point[1]/point[2]))
    else:
        rot_x=math.pi/3
    #return (math.pi/2-rot_x)
    #print('Print angles on x :'+str(rot_x)+" "+str(math.degrees(rot_x)))

    if(rot_x<0.5 and point[0]<0 and point[0]<0):
        rot_x=math.pi/4

    return rot_x

def compute_rot_z(point):


    if point[0]!=0.0:
    	rot_z=math.atan(abs(point[1]/point[0]))
    else:
        #rot_z=math.atan(abs(point[0]/1))
        if point[1]<0:
            rot_z=-math.pi/2
        else:
            rot_z=math.pi/2
    #print('Print angles on z :'+str(rot_z)+" "+str(math.degrees(rot_z)))
    return rot_z


def handle(req):
	print('We are here to make some noise!')
	center=req.center.position
	nr_of_points=req.nr_of_points.data
	radius=req.radius.data

	points=regular_on_sphere_points(radius,nr_of_points)

	#show_points(points)

	print('Sampled '+str(len(points))+' points!')

	#show_points(points)
	points=center_points(points,center)
	points=only(points,center)

	res=scanResponse()

	left = baxter_interface.Limb('left')

	v=0
	nv=0

	for point in points:
		
		print('\n\n\n-----\nMoving to :'+str(point)+'-----')

		offset_point=[]
		offset_point.append(point[0]-center.x)
		offset_point.append(point[1]-center.y)
		offset_point.append(point[2]-center.z)

		x=compute_rot_x(offset_point)
		z=compute_rot_z(offset_point)

		print(str(x)+' '+str(z)+' '+str(math.degrees(z)))
		

		if(point[0]>center.x and point[1]>center.y):
			final_z=-math.pi/2-z

		elif(point[0]>center.x and point[1]<center.y):
			final_z=math.pi/2+z

		elif(point[0]<center.x and point[1]<center.y):
			final_z=z
		else:
			final_z=-z

		if(point[1]==center.y and point[0]<center.x):
			final_z=0

		elif(point[1]==center.y and point[0]>center.x):
			final_z=math.pi

		final_x=x


		print('Angle Rad :'+str(final_x)+' 0 '+str(final_z))
		print('Angle:'+str(math.degrees(final_x))+' 0 '+str(math.degrees(final_z)))

		
		qq=quaternion_from_euler(0,math.pi-final_x,final_z)

		print('Tf quaternion :'+str(qq))

		ns = "ExternalTools/" + 'left'+ "/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq=SolvePositionIKRequest()
		hdr=Header(stamp=rospy.Time.now(),frame_id='world')

		pose=PoseStamped()
		pose.header=hdr

		pose.pose.position.x=point[0]
		pose.pose.position.y=point[1]
		pose.pose.position.z=point[2]


		#pose.pose.orientation.x=0.14
		#pose.pose.orientation.y=0.98
		#pose.pose.orientation.z=0.011
		#pose.pose.orientation.w=0.025

		pose.pose.orientation.x=qq[0]
		pose.pose.orientation.y=qq[1]
		pose.pose.orientation.z=qq[2]
		pose.pose.orientation.w=qq[3]

		ikreq.pose_stamp.append(pose)
		resp = iksvc(ikreq)

		if(resp.isValid[0]):
			#cloud_name='cloud_'+str(v)
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			left.move_to_joint_positions(limb_joints)

			v+=1

		else:
			nv+=1


		print('\n####Current status#####')
		print('Untill now, out of :'+str(len(points))+" valid :"+str(v)+" --- not valid :"+str(nv)+" unplanned :"+
			str(len(points)-nv-v))
		print('#########################\n')
	

		

		
	print('Valid :'+str(v)+' not valid :'+str(nv))


	return res




def main():
	#moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('bulb_scanner_node')
	server=rospy.Service('/bulb_scan',scan,handle)
	print('Bulb scanning server up . . .')
	rospy.spin()


if __name__ == '__main__':
	main()