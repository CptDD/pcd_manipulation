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

	elongated=[]
	livarno=[]
	mushroom=[]
	standard=[]

	elo_empty=True
	liv_empty=True
	mush_empty=True
	std_empty=True

	for i in range(0,len(files)):
		results=read_results(files[i],bulbs)

		print('For file :'+files[i]+' '+str(results['elongated']))

		if 'elongated' in bulbs:
			for j in range(0,len(results['elongated'])):
				if i==0:
					elongated.append(results['elongated'][j])
				else:
					elongated[j]+=results['elongated'][j]

		if 'livarno' in bulbs:
			for j in range(0,len(results['livarno'])):
				if i==0:
					livarno.append(results['livarno'][j])
				else:
					livarno[j]+=results['livarno'][j]


		if 'mushroom' in bulbs:
			for j in range(0,len(results['mushroom'])):
				if i==0:
					mushroom.append(results['mushroom'][j])
				else:
					mushroom[j]+=results['mushroom'][j]

		if 'standard' in bulbs:
			for j in range(0,len(results['standard'])):
				if i==0:
					standard.append(results['standard'][j])
				else:
					standard[j]+=results['standard'][j]


	if 'elongated' in bulbs:
		elongated=np.divide(elongated,len(files))

	if 'livarno' in bulbs:
		livarno=np.divide(livarno,len(files))

	if 'mushroom' in bulbs:
		mushroom=np.divide(mushroom,len(files))

	if 'standard' in bulbs:
		standard=np.divide(standard,len(files))


	plt.style.use('ggplot')
	fig=plt.figure()
	ax1 = fig.add_subplot(1,1,1)


	if 'elongated' in bulbs:
		ax1.plot(elongated,marker=r'o',color=u'red',linestyle='--',label='elongated')
	if 'livarno' in bulbs:
		ax1.plot(livarno,marker=r'o',color=u'blue',linestyle='--',label='livarno')
	if 'mushroom' in bulbs:
		ax1.plot(mushroom,marker=r'o',color=u'green',linestyle='--',label='mushroom')
	if 'standard' in bulbs:
		ax1.plot(standard,marker=r'o',color=u'yellow',linestyle='--',label='standard')

	ax1.xaxis.set_ticks_position('bottom')
	ax1.yaxis.set_ticks_position('left')
	ax1.set_title('Belief state evolution')
	plt.legend(loc='best')

	plt.show()









		


############################
path='/home/cptd/dd/despot/examples/pomdpx_models/results/'
fig_path='/home/cptd/c/'
bulbs=['elongated','livarno']

files=get_files(path)
process_files(files,bulbs,fig_path)

