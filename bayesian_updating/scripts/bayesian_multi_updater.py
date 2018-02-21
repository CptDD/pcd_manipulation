#!/usr/bin/python

import rospy
import rospkg
import numpy as np

from bayesian_updating.srv import *




class BayesianMultiUpdater:

	def __init__(self,node_name,service_name):
		self.node_name=node_name
		self.service_name=service_name
		self.server=rospy.Service(self.service_name,update,self.handler)
		self.distribution=dict()
		self.belief=list()


	def init_belief(self,number_of_objects):
		self.number_of_objects=number_of_objects

		value=1./self.number_of_objects

		self.belief=np.repeat(value,self.number_of_objects)

		print('Initial belief :'+str(self.belief))

	def set_number_of_objects(self,number_of_objects):
		self.number_of_objects=number_of_objects

	def get_number_of_objects(self):
		return self.number_of_objects

	def get_belief(self):
		return self.belief

	def print_test(self):
		print('We are here to make some noise!')

	def compute_entropy(self):
		print('Distribution entropy computation . . .')
		print(sum(self.belief))


	def fetch_distribution(self,file):
		f=open(file,'r')

		values=list()
		i=0

		for line in f:
			if '===' not in line:
				if i%6==0:
					viewpoint=line.split('/')[7]
				else:
					temp_tags=dict()
					positives=list()
					percentages=list()

					tags=line.split('\t')

					temp_tags['label']=tags[0]
					temp_tags['type']=tags[1]
					temp_tags['total']=tags[2]

					positives.append(tags[3])
					positives.append(tags[4])
					positives.append(tags[5])
					positives.append(tags[6])

					percentages.append(tags[7])
					percentages.append(tags[8])
					percentages.append(tags[9])
					percentages.append(tags[10])

					temp_tags['positives']=positives
					temp_tags['percentages']=percentages
					values.append(temp_tags)
			else:
				self.distribution[viewpoint]=values
				values=list()
			i+=1


	def update_belief(self,viewpoint,observed_type):

		temp_distribution=self.distribution[viewpoint]

		prior=np.array(self.belief)

		for i in temp_distribution:
			if int(i['type'])==observed_type:
				print(i['label'])
				print('Percentages '+str(i['percentages']))
				# observation vector corresponding to the training probability of the light bulb
				observation_vector=i['percentages']
				obs_vector=np.array(i['percentages'],dtype=float)

				print('Pre update belief :'+str(self.belief))
				self.belief*=obs_vector
				sum_b=np.sum(self.belief)
				self.belief/=sum_b

		posterior=np.array(self.belief)

		
		self.compute_kl_divergence(prior,posterior,observation_vector)


	def get_distribution(self):

		return self.distribution

	def compute_kl_divergence(self,prior,posterior,observation):
		pr=np.array(prior)
		ps=np.array(posterior)
		obs_vector=np.array(observation,dtype=float)
		#pr - prior probability
		#ps - posterior probaility
		#obs_vector observation probability

		self.information_gain=np.zeros(len(pr))


		print('The prior is :'+str(prior))
		print('The posterior is :'+str(posterior))


		for i in range(0,len(pr)):
			if pr[i]!=0 and ps[i]!=0:
				temp=ps[i]/pr[i]
				self.information_gain[i]=ps[i]*np.log2(temp)
				
		print("Information gain :"+str(self.information_gain))

		#eig expected information gain as a vector
		self.eig=obs_vector*self.information_gain

		#eig_sum scalar corresponding to the sum of the information gain
		self.eig_sum=np.sum(self.eig)

		print("Expectation :"+str(self.eig)+" "+"Sum :"+str(self.eig_sum))


	def get_eig(self):
		return self.eig

	def get_eig_sum(self):
		return self.eig_sum





	def handler(self,req):
		print('A request has been made')

		self.compute_entropy()

		response=updateResponse()

		if req.action.data==1:
			viewpoint=req.viewpoint.data
			observed_type=req.type.data
			self.update_belief(viewpoint,observed_type)
			print('Belief is :'+str(self.get_belief()))


		elif req.action.data==2:
			for i in self.belief:
				response.belief.append(i)
		else:
			response.eig=self.get_eig_sum()


		return response

	def start_server(self):
		rospy.init_node(self.node_name)

		print('Bayesian service up . . .')
		rospy.spin()


