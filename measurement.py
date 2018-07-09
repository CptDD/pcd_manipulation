#!/usr/bin/python

import glob
import numpy as np
import xml.etree.ElementTree as ET
from movement_plotter import *




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

def read_actions(tree):
	actions=[]

	for child in tree:
		if child.tag == 'Variable':
			for c in child:
				if c.tag == 'ActionVar':
					actions_text=c.find('ValueEnum').text
					actions=actions_text.rstrip().split(' ')
	return actions

def read_positions(tree):
	positions=[]

	for child in tree:
		if child.tag == 'Variable':
			for c in child:
				if c.tag == 'StateVar':
					if c.get('fullyObs') == 'true':
						positions_text=c.find('ValueEnum').text.replace('vp','').rstrip()
						positions=positions_text.split(' ')

	return positions


def associate_actions(unprocessed_actions,actions):

	processed_actions=[]
	for i in unprocessed_actions:
		action=int(i['action'])
		processed_actions.append(actions[action])


	print(processed_actions)

	return processed_actions


def get_points(tree):

	point=[]
	actions=[]
	nearest_point=[]
	points=dict()

	for child in tree:
		if child.tag == 'StateTransitionFunction':
			
			for c in child:
				if c.tag=='CondProb':
					var=c[0]
					if var.text=='robot_1':
						parameter=c[2]

						for entry in parameter:
							text=entry[0].text.replace('vp','')
							tokens=text.split(' ')

							action=tokens[0]
							point=tokens[1]
							neigh=tokens[2]

							if point not in points.keys():
								temp_tags=dict()
								temp_tags['south']=''
								temp_tags['east']=''
								temp_tags['north']=''
								temp_tags['west']=''
								points[point]=temp_tags

							points[point][action]=neigh

	return points

def prepare_positions(initial_pos,actions,points):

	prepared_positions=[]
	prepared_positions.append(initial_pos)

	current_pos=initial_pos

	print(initial_pos)

	for action in actions:
		new_pos=points[current_pos][action]
		print(new_pos)
		prepared_positions.append(new_pos)
		current_pos=new_pos

	return prepared_positions



						

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
	print(np.divide(elongated,norm_factor))
	print(np.divide(livarno,norm_factor))
	print(np.divide(mushroom,norm_factor))
	print(np.divide(standard,norm_factor))






### Files ###
results_filename='/home/cptd/dd/aems2/extended-appl-with-aems2/src/res.txt'
pomdpx_filename='/home/cptd/dd/aems2/extended-appl-with-aems2/examples/POMDPX/filename.pomdpx'
points_filename='/home/cptd/c/graph.txt'

elongated_filename='/home/cptd/test_results/gen_elongated.txt'
livarno_filename='/home/cptd/test_results/gen_livarno.txt'
mushroom_filename='/home/cptd/test_results/gen_mushroom.txt'
standard_filename='/home/cptd/test_results/gen_standard.txt'
bulbs=['elongated','livarno','mushroom','standard']

###############################################################################################

### Fetch needed data ###
tree = ET.parse(pomdpx_filename)
root = tree.getroot()
actions=read_actions(root)
positions=read_positions(root)

results,prior,posterior=read_results(results_filename)
associated_actions=associate_actions(results,actions)
points=get_points(root)
prepared_positions=prepare_positions('11',associated_actions,points)

#files=get_files('/home/cptd/dd/aems2/extended-appl-with-aems2/results')
#process_files(files)

print(prepared_positions)
print(associated_actions)

################################################################################################

### Data preparation ###
plotter=MovementPlotter(points_filename,elongated_filename,livarno_filename,mushroom_filename,standard_filename)
plotter.setup_grid()
plotter.setup_fig(step=False)
plotter.setup_entries(prepared_positions,associated_actions)
plotter.show(True,prepared_positions[0],prepared_positions[-1],True)



###############################################################################################








#########################


