#!/usr/bin/python

import rospy
import numpy as np

from conveyor_advance.srv import *
from gazebo_msgs.srv import *


def get_gazebo_info(nr_of_places):

	conveyor_positions=[]

	for i in range(0,nr_of_places):
		client=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
		req=GetModelStateRequest()
		req.model_name='bulb_'+str(i+1)
		res=client(req)
		position=res.pose
		conveyor_positions.append(position)

	#print(conveyor_positions)
	return conveyor_positions

def advance_bulbs(conveyor_positions):
	print('Advancing the bulbs')
	client=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)

	print(conveyor_positions)
	for i in range(0,len(conveyor_positions)-1):
		temp_y=conveyor_positions[i].position.y
		conveyor_positions[i].position.y=conveyor_positions[i+1].position.y
		conveyor_positions[i+1].position.y=temp_y

	print(conveyor_positions)


	for i in range(0,len(conveyor_positions)):
		
		req=SetModelStateRequest()
		req.model_state.model_name='bulb_'+str(i+1)
		req.model_state.pose=conveyor_positions[i]
		#req.model_state.pose.position.y=conveyor_positions[i-1].position.y
		#print('Conv pos')
		#new_y=conveyor_positions[i-1].position.y
		#req.model_state.pose.position.y=new_y
		res=client(req)
		#rospy.sleep(1.0)'''

	print('Done')




	


def handle(req):
	print('We are here to make some noise!')
	res=advanceResponse()
	res.msg.data='Advanced'
	conveyor_positions=get_gazebo_info(3)
	advance_bulbs(conveyor_positions)

	return res



def main():
	rospy.init_node('conveyor_advance_node')
	server=rospy.Service('conveyor_advance',advance,handle)
	print('Advance server up . . .')
	rospy.spin()

if __name__ == '__main__':
	main()