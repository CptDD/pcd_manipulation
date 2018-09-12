#!/usr/bin/env python


import numpy as np  
import glob
import matplotlib.pyplot as plt

def read_results(filename,bulbs):
	f=open(filename,'r')

	results=dict()
	elongated=[]
	livarno=[]
	mushroom=[]
	standard=[]

	pre=False
	post=False

	elongated.append(1./len(bulbs))
	livarno.append(1./len(bulbs))
	mushroom.append(1./len(bulbs))
	standard.append(1./len(bulbs))


	for line in f:
		'''if 'Pre' in line:
			pre=True'''

		if 'Post' in line:
			pre=False
			post=True

		if '---' in line:
			post=False

		if pre:
			if '[' in line:
				line=line.split('=')
				value=line[1].rstrip().lstrip()
				state=line[0].replace('[','').replace(']','')
				state=state.split(',')[1].split(':')[1]
				
				if 'elongated' in state:
					elongated.append(value)
				elif 'livarno' in state:
					livarno.append(value)
				elif 'mushroom' in state:
					mushroom.append(value)
				else:
					standard.append(value)
		if post:
			if '[' in line:
				line=line.split('=')
				value=float(line[1].rstrip().lstrip())
				state=line[0].replace('[','').replace(']','')
				state=state.split(',')[1].split(':')[1]

				if 'elongated' in state:
					elongated.append(value)
				elif 'livarno' in state:
					livarno.append(value)
				elif 'mushroom' in state:
					mushroom.append(value)
				else:
					standard.append(value)

	if 'elongated' in bulbs:
		results['elongated']=elongated

	if 'livarno' in bulbs:
		results['livarno']=livarno

	if 'mushroom' in bulbs:
		results['mushroom']=mushroom

	if 'standard' in bulbs:
		results['standard']=standard

	return results


def get_files(path):
	return glob.glob(str(path+'/*.txt'))

def process_files(files,bulbs,fig_path):

	plt.style.use('ggplot')
	i=0
	for file in files:
		results=read_results(file,bulbs)

		temp_fig=plt.figure()
		ax1=temp_fig.add_subplot(1,1,1)


		if 'elongated' in bulbs:
			ax1.plot(results['elongated'],marker=r'o',color=u'red',linestyle='--',label='elongated')
		if 'livarno' in bulbs:
			ax1.plot(results['livarno'],marker=r'o',color=u'blue',linestyle='--',label='livarno')
		if 'mushroom' in bulbs:
			ax1.plot(results['mushroom'],marker=r'o',color=u'green',linestyle='--',label='mushroom')
		if 'standard' in bulbs:
			ax1.plot(results['standard'],marker=r'o',color=u'yellow',linestyle='--',label='standard')

		ax1.xaxis.set_ticks_position('bottom')
		ax1.yaxis.set_ticks_position('left')
		ax1.set_title('Belief state evolution')
		plt.legend(loc='best')

		fig_name=fig_path+str(i)+'.png'
		temp_fig.savefig(fig_name, dpi=temp_fig.dpi)
		i+=1

	plt.show()






		


############################
path='/home/cptd/dd/despot/examples/pomdpx_models/results/'
fig_path='/home/cptd/c/'
bulbs=['elongated','livarno']

files=get_files(path)
process_files(files,bulbs,fig_path)

