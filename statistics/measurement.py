#!/usr/bin/python

import glob
import numpy as np
import xml.etree.ElementTree as ET
from movement_plotter import *
import matplotlib.pyplot as plt


def read_results(filename):
	f=open(filename,'r')

	results=[]
	prior=[]
	posterior=[]

	for line in f:
		if '-simualtion step:' in line:
			line=line.lstrip().rstrip().split(' ')
			temp_tags['action']=line[4]
			temp_tags['observation']=line[6]
			temp_tags['state']=line[8]

		if 'State:' in line:
			print(line)

		if 'Reward :' in line:
			temp_tags=dict()
			line=line.replace('Reward :','')
			temp_tags['reward']=line.rstrip()

		if 'Initializing pre' in line:
			temp_prior=dict()
			line=line.replace('Initializing pre!','')
			line=line.lstrip().rstrip().split(' ')
			temp_prior['elongated']=line[1]
			temp_prior['livarno']=line[3]
			temp_prior['mushroom']=line[5]
			temp_prior['standard']=line[7]

		if 'Initializing post' in line:
			temp_posterior=dict()
			line=line.replace('Initializing post!','')
			line=line.lstrip().rstrip().split(' ')
			temp_posterior['elongated']=line[1]
			temp_posterior['livarno']=line[3]
			temp_posterior['mushroom']=line[5]
			temp_posterior['standard']=line[7]

		if '=*=*=' in line:
			results.append(temp_tags)
			prior.append(temp_prior)
			posterior.append(temp_posterior)


	f.close()
	return results,prior,posterior


def compute_mean_values(posterior):

	elo_sum=0
	liv_sum=0
	mush_sum=0
	std_sum=0 

	for i in posterior:
		elo_sum+=float(i['elongated'])
		liv_sum+=float(i['livarno'])
		mush_sum+=float(i['mushroom'])
		std_sum+=float(i['standard'])

	print(elo_sum)
	print(liv_sum)
	print(mush_sum)
	print(std_sum)

def get_files(path):
	return glob.glob(str(path+'/*.txt'))

def process_files(files):

	elongated=[]
	livarno=[]
	mushroom=[]
	standard=[]

	for i in range(0,len(files)):
		f=files[i]
		print('Filename :'+f)
		res,pre,post=read_results(f)

		if i==0:
			first=True
		else:
			first=False

		for j in range(0,len(pre)):
			if first:
				elongated.append(float(pre[j]['elongated']))
				livarno.append(float(pre[j]['livarno']))
				mushroom.append(float(pre[j]['mushroom']))
				standard.append(float(pre[j]['standard']))
			else:
				elongated[j]+=float(pre[j]['elongated'])
				livarno[j]+=float(pre[j]['livarno'])
				mushroom[j]+=float(pre[j]['mushroom'])
				standard[j]+=float(pre[j]['standard'])

		if first:
			elongated.append(float(post[-1]['elongated']))
			livarno.append(float(post[-1]['livarno']))
			mushroom.append(float(post[-1]['mushroom']))
			standard.append(float(post[-1]['standard']))
			first=False

		else:
			elongated[-1]+=float(post[-1]['elongated'])
			livarno[-1]+=float(post[-1]['livarno'])
			mushroom[-1]+=float(post[-1]['mushroom'])
			standard[-1]+=float(post[-1]['standard'])


	norm_factor=len(files)
	elo=np.divide(elongated,norm_factor)
	liv=np.divide(livarno,norm_factor)
	mush=np.divide(mushroom,norm_factor)
	std=np.divide(standard,norm_factor)


	print(np.divide(elongated,norm_factor))
	print(np.divide(livarno,norm_factor))
	print(np.divide(mushroom,norm_factor))
	print(np.divide(standard,norm_factor))

	return elo,liv,mush,std






### Files ###
results_filename='/home/cptd/dd/aems2/extended-appl-with-aems2/src/res.txt'
pomdpx_filename='/home/cptd/dd/aems2/extended-appl-with-aems2/examples/POMDPX/filename.pomdpx'
points_filename='/home/cptd/c/graph.txt'

path='/home/cptd/dd/aems2/extended-appl-with-aems2/results/'

###############################################################################################

### Fetch needed data ###
files=get_files(path)
elo,liv,mush,std=process_files(files)

plt.style.use('ggplot')



fig=plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax1.plot(elo,marker=r'o',color=u'red',linestyle='--',label='Elongated')
ax1.plot(liv,marker=r'o',color=u'blue',linestyle='--',label='Livarno')
ax1.plot(mush,marker=r'o',color=u'yellow',linestyle='--',label='Mushroom')
ax1.plot(std,marker=r'o',color=u'green',linestyle='--',label='Standard')
ax1.xaxis.set_ticks_position('bottom')
ax1.yaxis.set_ticks_position('left')
ax1.set_title('Belief state evolution standard')
plt.legend(loc='best')
plt.show()





################################################################################################


