#!/usr/bin/python


from bayesian_updater import *

import rospkg


def update_handle(req):

	'''rospack=rospkg.RosPack()
	print('Here')
	updater=BayesianUpdater()

	path=rospack.get_path('pcd_processor')
	path+='/training_results/out.txt'

	updater.fetch(path)
	print(updater.get_distro())

	return updateResponse()'''



def main():
	b=BayesianUpdater('bay_up','bay_service')

	rospack=rospkg.RosPack()

	path=rospack.get_path('pcd_processor')
	path+='/training_results/out.txt'

	b.fetch_distribution(path)
	b.init_belief(4)
	b.start_server()



if __name__=='__main__':
	main()