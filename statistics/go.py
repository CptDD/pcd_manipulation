#!/usr/bin/python

import math
import numpy as np


def get_custom_sequence():

	actions=[]
	observations=[]

	actions.append('top')
	observations.append(0)

	actions.append('top')
	observations.append(0)

	actions.append('top')
	observations.append(1)

	actions.append('top')
	observations.append(2)

	actions.append('top')
	observations.append(2)

	actions.append('top')
	observations.append(0)

	actions.append('top')
	observations.append(3)

	actions.append('top')
	observations.append(0)

	actions.append('top')
	observations.append(1)

	actions.append('top')
	observations.append(0)




	'''for i in range(0,len(actions)):
		print(actions[i]+" "+str(observations[i]))'''

	return actions,observations



def process_line(line):
	print("Original line :"+line)


def custom_f():
	f=open("out_test.txt","r")

	distro=dict()

	values=[]

	i=0

	for line in f:
		if "===" not in line:
			if i%4==0:
				viewpoint=line.split('/')[7]
			else:
				temp_tags=dict()
				percentages=list()
				positives=list()

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

				temp_tags['positives']=positives
				temp_tags['percentages']=percentages
				values.append(temp_tags)
		else:
			distro[viewpoint]=values
			values=list()

		i+=1


	return distro


def f():
	f=open("out_multiple.txt","r")

	distro=dict()

	values=[]

	i=0

	for line in f:
		if "===" not in line:
			if i%6==0:
				viewpoint=line.split('/')[7]
			else:
				temp_tags=dict()
				percentages=list()
				positives=list()

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
			distro[viewpoint]=values
			values=list()

		i+=1

	return distro

def fetch():
	f=open("out_test.txt","r")

	distro=dict()

	values=[]
	i=0
	for line in f:
		if "===" not in line:
			if i%6==0:
				viewpoint=line.split('/')[7]
			else:
				temp_tags=dict()
				tags=line.split('\t')
				temp_tags['label']=tags[0]
				temp_tags['type']=tags[1]
				temp_tags['positives']=tags[2]
				temp_tags['total']=tags[3]
				temp_tags['percentage']=tags[4].split('\n')[0]
				values.append(temp_tags)
		else:
			distro[viewpoint]=values
			values=list()
		i+=1

	return distro


def compute_kl(posterior,prior,observation):
	print('KL distance')
	print('Posterior'+str(posterior))
	print('Prior '+str(prior))

	pr=np.array(prior)
	ps=np.array(posterior)
	print('Observation :'+str(observation))

	print(pr)
	print(ps)


	temp=pr/ps

	print('Division :'+str(temp))

	if sum(prior)==0:
		information_gain=0

	if sum(posterior)==0:
		information_gain=1



	information_gain=posterior*np.log2(temp)
	#eig=-(information_gain*observation)
	eig=np.sum(information_gain)*observation

	print('Information gain :'+str(information_gain)+" "+str(np.sum(information_gain)))
	print(eig)

	



def main():
	#update()
	
	custom_update()


def custom_update():
	distribution=f()

	print(distribution)

	belief=np.repeat(0.25,4)
	print('---Initial belief---')
	print(belief)

	actions,observations=get_custom_sequence()

	for i in range(0,len(actions)):

		observed_type=observations[i]
		observed_viewpoint=actions[i]


		prior=list(belief)

		temp=distribution[observed_viewpoint]

	
		print('Perceee: '+str(temp[observed_type]['percentages']))

		print('Action :'+observed_viewpoint+' Observation :'+str(observed_type))

		print('---Pre---')
		print(belief)

		for j in temp:

			if int(j['type'])==observed_type:
				obs=np.array(j['percentages'],dtype=float)
				print('Percentages :'+str(obs))
				belief*=obs
				sum_b=np.sum(belief)
				belief/=sum_b

		print('---Post---')
		print(belief)

		




def update():
	distribution=f()

	print(distribution)

	belief=np.repeat(0.25,4)
	#belief=[0.25,0.25,0.25,0.25]
	print(belief)


	'''Test values for observed type and observed viewpoint'''

	observed_type=2
	observed_viewpoint="side_right"

	temp=distribution[observed_viewpoint]

	prior=list(belief)

	for i in temp:

		if int(i['type'])==observed_type:
			print(i['label'])
			print(i['percentages'])
			obs=np.array(i['percentages'],dtype=float)
			belief*=obs

			print(belief)

			sum_b=np.sum(belief)

			print(sum_b)

			belief/=sum_b

			print(belief)



			
			'''val=float(i['percentage'])*belief[observed_type]
			belief[observed_type]=val
			sum_b=sum(belief)

			for j in range(0,4):
				belief[j]/=sum_b'''
    
	posterior=list(belief)
	
	'''print(temp)
	percentage=float(temp[observed_type]['percentage'])
	print(percentage)

	compute_kl(posterior,prior,percentage)'''




				

if __name__=='__main__':
	main()