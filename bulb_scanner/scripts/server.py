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

def compute_quaternion(point,center):

	offset_point=[]
	offset_point.append(point[0]-center.x)
	offset_point.append(point[1]-center.y)
	offset_point.append(point[2])

	x=compute_rot_x(offset_point)
	z=compute_rot_z(offset_point)

	if(point[0]>center.x and point[1]>center.y):
		final_z=math.pi-z
	elif(point[0]>center.x and point[1]<center.y):
		final_z=z
	elif(point[0]<center.x and point[1]<center.y):
		final_z=-z
	else:
		final_z=math.pi+z

	if(point[1]==center.y):
		final_z=math.pi/2

	final_x=x

	#print('Angle Rad :'+str(final_x)+' 0 '+str(final_z))
	#print('Angle:'+str(math.degrees(final_x))+' 0 '+str(math.degrees(final_z)))

	#quat=euler2quat(final_z,0,final_x)
	quat=quaternion_from_euler(0,0,final_z);

	return quat



def points_2_msg(points,center):

	msg=[]

	for point in points:

		quat=compute_quaternion(point,center)

		temp=Pose()
		temp.position.x=point[0]
		temp.position.y=point[1]
		temp.position.z=point[2]

		temp.orientation.w=quat[3]
		temp.orientation.x=quat[0]
		temp.orientation.y=quat[1]
		temp.orientation.z=quat[2]

		msg.append(temp)
	return msg

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


def euler2quat(z=0, y=0, x=0):
	
    cy = math.cos(z * 0.5);
    sy = math.sin(z * 0.5);
    cr = math.cos(x * 0.5);
    sr = math.sin(x * 0.5);
    cp = math.cos(y * 0.5);
    sp = math.sin(y * 0.5);

    w = cy * cr * cp + sy * sr * sp;
    x = cy * sr * cp - sy * cr * sp;
    y = cy * cr * sp + sy * sr * cp;
    z = sy * cr * cp - cy * sr * sp;


    return np.array([w,x,y,z])

    '''z = z/2.0
    y = y/2.0
    x = x/2.0
    cz = math.cos(z)
    sz = math.sin(z)
    cy = math.cos(y)
    sy = math.sin(y)
    cx = math.cos(x)
    sx = math.sin(x)
    return np.array([
             cx*cy*cz - sx*sy*sz,
             cx*sy*sz + cy*cz*sx,
             cx*cz*sy - sx*cy*sz,
	cx*cy*sz + sx*cz*sy])'''

def label_points(points):
	labeled_points=dict()
	#unique=uuid.uuid1()
	for i in range(len(points)):
		labeled_points['p'+str(i)]=points[i]
	return labeled_points

def create_graph(points):

	graph=[]

	for k,v in points.items():
		elem=dict()
		org=[]
		org.append(v)
		elem['original']=org
		elem['label']=k

		nearest_points,nearest_labels=get_labeled_nearest_neighbours(v,points,0.5)

		elem['neighbour_points']=nearest_points
		elem['neighbour_labels']=nearest_labels

		graph.append(elem)

	return graph

def get_labeled_nearest_neighbours(point,points,radius):

	neighbour_points=[]
	neighbour_labels=[]

	for k,v in points.items():
		if is_nearest_neighbour(point,v,radius) and point!=v:

			neighbour_points.append(v)
			neighbour_labels.append(k)

	return neighbour_points,neighbour_labels


def is_nearest_neighbour(point,candidate,radius):

	if(candidate[0]>=point[0]-radius and candidate[0]<=point[0]+radius) and (candidate[1]>=point[1]-radius and candidate[1]<=point[1]+radius and candidate[2]>=point[2]-radius and candidate[2]<=point[2]+radius):
		return True
	else:
		return False

def labeled_graph_to_file(graph,filename):
	out=open(filename,'w')

	for i in range(0,len(graph)):
		temp=graph[i]
		neigh_points=temp['neighbour_points']
		neigh_labels=temp['neighbour_labels']

		out.write('Original '+temp['label']+'\n')
		out.write(str(temp['original'][0][0])+' '+str(temp['original'][0][1])+' '+str(temp['original'][0][2])+'\n')
		out.write('_______________\n')
		out.write('Neighbours\n')
		for j in range(len(neigh_points)):
			out.write(str(j+1)+' '+neigh_labels[j]+' ')
			out.write(str(neigh_points[j][0])+'\t'+str(neigh_points[j][1])+'\t'+str(neigh_points[j][2])+'\n')
		out.write('_______________\n')
		out.write('---\n')
	out.close()

def valid_points_to_file(points,filename):
	out=open(filename,'w')

	for i in range(0,len(points)):
		temp=points[i]

		neigh_points=temp['neighbour_points']
		neigh_labels=temp['neighbour_labels']

		out.write('Original '+temp['label']+'\n')
		out.write('Position :'+str(temp['original'][0])+'\t'+str(temp['original'][1])+'\t'+str(temp['original'][2])+'\n')
		out.write('Orientation :'+str(temp['orientation'][0])+'\t'+str(temp['orientation'][1])+'\t'+str(temp['orientation'][2])+'\t'+str(temp['orientation'][3])+'\n')
		out.write('_______________\n')
		out.write('Neighbours\n')

		for j in range(len(neigh_points)):
			out.write(str(j+1)+' '+neigh_labels[j]+' ')
			out.write(str(neigh_points[j][0])+'\t'+str(neigh_points[j][1])+'\t'+str(neigh_points[j][2])+'\n')
			out.write('_______________\n')
		out.write('---\n')

	out.close()


      

def save_cloud(cloud_name):

	cloudSaverClient=rospy.ServiceProxy('/pcd_cloud_saver_service',save)

	cloudReq=saveRequest()
	cloudReq.cloud_name.data=cloud_name

	cloudSaverClient(cloudReq)

def handle(req):
	print('We are here to make some noise!')
	center=req.center.position
	nr_of_points=req.nr_of_points.data
	radius=req.radius.data

	valid_points=[]

	#print(center)
	#print(nr_of_points)
	#print(radius)

	points=regular_on_sphere_points(radius,nr_of_points)

	#show_points(points)

	print('Sampled '+str(len(points))+' points!')

	#show_points(points)
	points=center_points(points,center)
	points=only(points,center)

	labeled_points=label_points(points)
	graph=create_graph(labeled_points)

	res=scanResponse()
	res.points=points_2_msg(points,center)

	'''robot=moveit_commander.RobotCommander()
	group = moveit_commander.MoveGroupCommander('left_arm')
	print(robot.get_current_state())
	#robot.set_current_state(robot.get_current_state())'''


	f=open('/home/cptd/.gazebo/models/bulb_marker/model.sdf','r')
	model_xml_no = f.read()

	f=open('/home/cptd/.gazebo/models/bulb_marker_good/model.sdf','r')
	model_xml_good=f.read()


	left = baxter_interface.Limb('left')

	valid_point_count=0
	non_valid_point_count=0

	valid_points=[]

	for point in graph:

		print('\n\n\n-----\nMoving to :'+str(point['label'])+' '+str(point['original'])+'-----')

		offset_point=[]
		offset_point.append(float(point['original'][0][0])-center.x)
		offset_point.append(float(point['original'][0][1])-center.y)
		offset_point.append(float(point['original'][0][2])-center.z)

		x=compute_rot_x(offset_point)
		z=compute_rot_z(offset_point)

		if(point['original'][0][0]>center.x and point['original'][0][1]>center.y):
			final_z=-math.pi/2-z

		elif(point['original'][0][0]>center.x and point['original'][0][1]<center.y):
			final_z=math.pi/2+z

		elif(point['original'][0][0]<center.x and point['original'][0][1]<center.y):
			final_z=z
		else:
			final_z=-z

		if(point['original'][0][1]==center.y and point['original'][0][0]<center.x):
			final_z=0

		elif(point['original'][0][1]==center.y and point['original'][0][0]>center.x):
			final_z=math.pi

		final_x=x

		quat=quaternion_from_euler(0,math.pi-final_x,final_z)

		print('Computed quaterion :'+str(quat))

		ns = "ExternalTools/" + 'left'+ "/PositionKinematicsNode/IKService"
		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq=SolvePositionIKRequest()
		hdr=Header(stamp=rospy.Time.now(),frame_id='world')

		pose=PoseStamped()
		pose.header=hdr

		pose.pose.position.x=point['original'][0][0]
		pose.pose.position.y=point['original'][0][1]
		pose.pose.position.z=point['original'][0][2]

		#pose.pose.orientation.x=0.14
		#pose.pose.orientation.y=0.98
		#pose.pose.orientation.z=0.011
		#pose.pose.orientation.w=0.025

		pose.pose.orientation.x=quat[0]
		pose.pose.orientation.y=quat[1]
		pose.pose.orientation.z=quat[2]
		pose.pose.orientation.w=quat[3]

		ikreq.pose_stamp.append(pose)
		resp = iksvc(ikreq)

		if(resp.isValid[0]):
			#cloud_name='cloud_'+str(v)
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			left.move_to_joint_positions(limb_joints)

			temp_point=dict()
			temp_point['original']=point
			temp_point['orientation']=quat
			temp_point['label']=point['label']

			valid_points.append(temp_point)

			valid_point_count+=1
			for k in range(0,10):
				cloud_name='cloud_'+point['label']+'_'+str(k)
				save_cloud(cloud_name)
			#input('Enter something')
		else:
			non_valid_point_count+=1

		print('\n####Current status#####')
		print('Untill now, out of :'+str(len(graph))+" valid :"+str(valid_point_count)+" --- not valid :"+str(non_valid_point_count)+" unplanned :"+
			str(len(graph)-non_valid_point_count-valid_point_count))
		print('#########################\n')
	

		#spawn_client=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)

		#model=SpawnModelRequest()
		#model.model_name=str(point)
		#model.reference_frame='base'


		#print('FFF '+str(round(point[0],2))+' '+str(round(point[1],2))+' '+str(round(point[2],2)))

		#model.initial_pose.position.x=round(point[0],2)
		#model.initial_pose.position.y=round(point[1],2)
		#model.initial_pose.position.z=round(point[2],2)

		#model.initial_pose.orientation.w=1
		#model.initial_pose.orientation.x=0
		#model.initial_pose.orientation.y=0
		#model.initial_pose.orientation.z=0



		#if(resp.isValid[0]):
		#	print('Pose is valid')
		#	model.model_xml=model_xml_good
		#else:
		#	print('Pose is not valid')
		#	model.model_xml=model_xml_no

		#spawn_client(model)

	print('Valid :'+str(valid_point_count)+' not valid :'+str(non_valid_point_count))

	'''for i in range(0,len(valid_points)):

		print('Moving to starting point:'+str(valid_points[i]['label']))

		neighbour_points=[]
		neighbour_labels=[]


		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq=SolvePositionIKRequest()
		hdr=Header(stamp=rospy.Time.now(),frame_id='world')

		pose=PoseStamped()
		pose.header=hdr

		pose.pose.position.x=valid_points[i]['original'][0]
		pose.pose.position.z=valid_points[i]['original'][1]
		pose.pose.position.y=valid_points[i]['original'][2]

		pose.pose.orientation.x=valid_points[i]['orientation'][0]
		pose.pose.orientation.y=valid_points[i]['orientation'][1]
		pose.pose.orientation.z=valid_points[i]['orientation'][2]
		pose.pose.orientation.w=valid_points[i]['orientation'][3]

		ikreq.pose_stamp.append(pose)
		resp = iksvc(ikreq)


		if(resp.isValid[0]):
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			left.move_to_joint_positions(limb_joints)

		for j in range(0,len(valid_points)):
			if i!=j:
				print('Moving to temp point :'+str(valid_points[j]['label']))

				iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
				ikreq=SolvePositionIKRequest()
				hdr=Header(stamp=rospy.Time.now(),frame_id='world')

				pose=PoseStamped()
				pose.header=hdr

				pose.pose.position.x=valid_points[j]['original'][0]
				pose.pose.position.z=valid_points[j]['original'][1]
				pose.pose.position.y=valid_points[j]['original'][2]


				pose.pose.orientation.x=valid_points[j]['orientation'][0]
				pose.pose.orientation.y=valid_points[j]['orientation'][1]
				pose.pose.orientation.z=valid_points[j]['orientation'][2]
				pose.pose.orientation.w=valid_points[j]['orientation'][3]

				ikreq.pose_stamp.append(pose)
				resp = iksvc(ikreq)

				if(resp.isValid[0]):
					limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
					left.move_to_joint_positions(limb_joints)

					print('Valid from :'+str(valid_points[i]['label'])+' to '+str(valid_points[j]['label']))

					neighbour_points.append(valid_points[j]['original'])
					neighbour_labels.append(valid_points[j]['label'])

				else:
					print('Not valid from :'+str(valid_points[i]['label'])+' to '+str(valid_points[j]['label']))


		valid_points[i]['neighbour_points']=neighbour_points
		valid_points[i]['neighbour_labels']=neighbour_labels'''




	valid_points_filename='/home/cptd/go_ws/src/bulb_scanner/graph/valid.txt'
	valid_points_to_file(valid_points,valid_points_filename)

	graph_filename='/home/cptd/go_ws/src/bulb_scanner/graph/graph.txt'
	labeled_graph_to_file(graph,graph_filename)





	''''for point in points:
		
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

		quat=euler2quat(final_z,0,final_x)
		qq=quaternion_from_euler(0,math.pi-final_x,final_z)

		print('Computed quaterion :'+str(quat))
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

			temp_point=dict()
			temp_point['original']=point
			temp_point['orientation']=qq
			temp_point['label']='p'+str(v)

			valid_points.append(temp_point)

			v+=1
			for k in range(0,10):
				cloud_name='cloud_'+str(v)+'_'+str(k)
				save_cloud(cloud_name)
			#input('Enter something')
		else:
			nv+=1


		print('\n####Current status#####')
		print('Untill now, out of :'+str(len(points))+" valid :"+str(v)+" --- not valid :"+str(nv)+" unplanned :"+
			str(len(points)-nv-v))
		print('#########################\n')
	

		#spawn_client=rospy.ServiceProxy('/gazebo/spawn_sdf_model',SpawnModel)

		#model=SpawnModelRequest()
		#model.model_name=str(point)
		#model.reference_frame='base'


		#print('FFF '+str(round(point[0],2))+' '+str(round(point[1],2))+' '+str(round(point[2],2)))

		#model.initial_pose.position.x=round(point[0],2)
		#model.initial_pose.position.y=round(point[1],2)
		#model.initial_pose.position.z=round(point[2],2)

		#model.initial_pose.orientation.w=1
		#model.initial_pose.orientation.x=0
		#model.initial_pose.orientation.y=0
		#model.initial_pose.orientation.z=0



		#if(resp.isValid[0]):
		#	print('Pose is valid')
		#	model.model_xml=model_xml_good
		#else:
		#	print('Pose is not valid')
		#	model.model_xml=model_xml_no

		#spawn_client(model)

		
	print('Valid :'+str(v)+' not valid :'+str(nv))
	print('Valid points :'+str(valid_points))'''

	'''for i in range(0,len(valid_points)):

		print('Moving to starting point:'+str(valid_points[i]['label']))

		neighbour_points=[]
		neighbour_labels=[]


		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq=SolvePositionIKRequest()
		hdr=Header(stamp=rospy.Time.now(),frame_id='world')

		pose=PoseStamped()
		pose.header=hdr

		pose.pose.position.x=valid_points[i]['original'][0]
		pose.pose.position.z=valid_points[i]['original'][1]
		pose.pose.position.y=valid_points[i]['original'][2]

		pose.pose.orientation.x=valid_points[i]['orientation'][0]
		pose.pose.orientation.y=valid_points[i]['orientation'][1]
		pose.pose.orientation.z=valid_points[i]['orientation'][2]
		pose.pose.orientation.w=valid_points[i]['orientation'][3]

		ikreq.pose_stamp.append(pose)
		resp = iksvc(ikreq)


		if(resp.isValid[0]):
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			left.move_to_joint_positions(limb_joints)

		for j in range(0,len(valid_points)):
			if i!=j:
				print('Moving to temp point :'+str(valid_points[j]['label']))

				iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
				ikreq=SolvePositionIKRequest()
				hdr=Header(stamp=rospy.Time.now(),frame_id='world')

				pose=PoseStamped()
				pose.header=hdr

				pose.pose.position.x=valid_points[j]['original'][0]
				pose.pose.position.z=valid_points[j]['original'][1]
				pose.pose.position.y=valid_points[j]['original'][2]


				pose.pose.orientation.x=valid_points[j]['orientation'][0]
				pose.pose.orientation.y=valid_points[j]['orientation'][1]
				pose.pose.orientation.z=valid_points[j]['orientation'][2]
				pose.pose.orientation.w=valid_points[j]['orientation'][3]

				ikreq.pose_stamp.append(pose)
				resp = iksvc(ikreq)

				if(resp.isValid[0]):
					limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
					left.move_to_joint_positions(limb_joints)

					print('Valid from :'+str(valid_points[i]['label'])+' to '+str(valid_points[j]['label']))

					neighbour_points.append(valid_points[j]['original'])
					neighbour_labels.append(valid_points[j]['label'])

				else:
					print('Not valid from :'+str(valid_points[i]['label'])+' to '+str(valid_points[j]['label']))


		valid_points[i]['neighbour_points']=neighbour_points
		valid_points[i]['neighbour_labels']=neighbour_labels




	valid_points_filename='/home/cptd/go_ws/src/bulb_scanner/graph/valid.txt'
	valid_points_to_file(valid_points,valid_points_filename)

	graph_filename='/home/cptd/go_ws/src/bulb_scanner/graph/graph.txt'
	labeled_graph_to_file(graph,graph_filename)'''

	return res




def main():
	#moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('bulb_scanner_node')
	server=rospy.Service('/bulb_scan',scan,handle)
	print('Bulb scanning server up . . .')
	rospy.spin()


if __name__ == '__main__':
	main()