#!/usr/bin/python

from movement_plotter import *




### Utility functions ###

def read_results(filename):
	f=open(filename,'r')

	actions=[]
	positions=[]

	for line in f:
		if 'Action =' in line:
			temp_action=line.split(':')[1].rstrip()
			actions.append(temp_action)
		if 'State =' in line:
			if 'Initial' in line:
				line=line.replace('Initial State =','').lstrip()
			else:
				line=line.replace('- State =','').lstrip()
			line=line.replace('[','')
			line=line.replace(']','')
			line=line.split(', ')[0]
			line=line.split(':')[1].replace('vp','')
			positions.append(line)


	f.close()

	return positions,actions

#######################################


########## Fetch needed data ##########

points_filename='/home/cptd/c/graph_test.txt'
elongated_filename='/home/cptd/test_results/gen_elongated.txt'
livarno_filename='/home/cptd/test_results/gen_livarno.txt'
mushroom_filename='/home/cptd/test_results/gen_mushroom.txt'
standard_filename='/home/cptd/test_results/gen_standard.txt'
#results_filename='/home/cptd/dd/despot/examples/pomdpx_models/res.txt'
results_filename='/home/cptd/c/result.txt'
#bulbs=['elongated','livarno','mushroom','standard']
bulbs=['elongated','livarno']

positions=['5','5','5','6','6','6','6','6','6']
actions=['east','north','west','decision','decision','west','north','decision']
#positions,actions=read_results(results_filename)


print(positions)
print(actions)

plotter=MovementPlotter(points_filename,elongated_filename,livarno_filename,mushroom_filename,standard_filename)
plotter.setup_grid()
plotter.setup_fig(step=False)
plotter.setup_entries(positions,actions)
plotter.show(True,positions[0],positions[-1],True)

######################################

