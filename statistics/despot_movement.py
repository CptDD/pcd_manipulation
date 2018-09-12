#!/usr/bin/python

import glob
import numpy as np
import xml.etree.ElementTree as ET
from movement_plotter import *







def read_results(filename):
	f=open(filename,'r')

	actions=[]
	positions=[]

	for line in f:
		if 'Action =' in line:
			temp_action=line.split(':')[1].rstrip()
			actions.append(temp_action)

		if 'Initial' in line:
			line=line.replace('Initial state:','').lstrip()
			line=line.replace('- State:','').lstrip()
			line=line.replace('[','')
			line=line.replace(']','')
			line=line.split(', ')[0]
			line=line.split(':')[1].replace('vp','')
			positions.append(line)

		if '- State:' in line:
			line=line.replace('- State:','').lstrip()
			line=line.replace('[','')
			line=line.replace(']','')
			values=line.split(', ')
			pos=values[0]
			state=values[1].split(':')[1].rstrip()
			pos=pos.split(':')[1].replace('vp','')
			positions.append(pos)


	f.close()

	return positions,actions,state




### Files ###
#results_filename='/home/cptd/c/result.txt'
results_filename='/home/cptd/c/rr.txt'
points_filename='/home/cptd/c/graph_test.txt'

elongated_filename='/home/cptd/test_results/gen_elongated.txt'
livarno_filename='/home/cptd/test_results/gen_livarno.txt'
mushroom_filename='/home/cptd/test_results/gen_mushroom.txt'
standard_filename='/home/cptd/test_results/gen_standard.txt'
bulbs=['elongated','livarno']
#bulbs=['elongated','livarno','mushroom','standard']

positions,actions,state=read_results(results_filename)


###############################################################################################

### Data preparation ###
plotter=MovementPlotter(points_filename,elongated_filename,livarno_filename,mushroom_filename,standard_filename)
plotter.setup_grid()
plotter.setup_fig(step=False)
plotter.setup_entries(positions,actions)
plotter.show(True,positions[0],positions[-1],True)



####################################################################################################