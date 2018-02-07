#!/usr/bin/python

import rospy

from bayesian_updating.srv import *
from bay_update import *

import rospkg

def update_handle(req):
	rospack=rospkg.RosPack()
	print('Here')
	updater=BayesianUpdater()

	path=rospack.get_path('pcd_processor')
	path+='/training_results/out.txt'

	updater.fetch(path)
	print(updater.get_distro())

	return updateResponse()



def main():
	rospy.init_node('bayesian_updating_server')
	server=rospy.Service('bayesian_updating',update,update_handle)
	print('Service up . . .')
	rospy.spin()

if __name__=='__main__':
	main()