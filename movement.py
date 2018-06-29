#!/usr/bin/python

import xml.etree.cElementTree as ET

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def read_points(filename):
    f=open(filename,'r')

    points=dict()

    for line in f:
        if '---' not in line:
            if 'Original' in line:
                temp_tags=dict()
                label=line.split(' ')[1]
                x=line.split(' ')[2]
                y=line.split(' ')[3]
                z=line.split(' ')[4]

                #temp_tags['label']=label
                temp_tags['x']=x
                temp_tags['y']=y
                temp_tags['z']=z

            elif 'Neighbours' in line:
            	print(line)
            	neighbours=[]
            	temp=line.split(' ')
            	for i in range(1,len(temp)):
            		neighbours.append(temp[i].strip())

            	temp_tags['neighbours']=neighbours

            else:
                points[label]=temp_tags
        
    print(points)

    f.close()
    return points

def read_percentages(filename):
    f=open(filename,'r')
    percs=dict()

    i=0

    for line in f:
        if '---' not in line:
            if 'Label:' in line:
                label=line.split('\t\t')[1].strip()
            if 'Percentage_e:' in line:
                percentage_e=line.split('\t')[1].strip()
            if 'Percentage_l:' in line:
            	percentage_l=line.split('\t')[1].strip()
            if 'Percentage_m:' in line:
            	percentage_m=line.split('\t')[1].strip()
            if 'Percentage_s' in line:
            	percentage_s=line.split('\t')[1].strip()


        else:
        	temp_percs=dict()
        	temp_percs['elongated']=percentage_e
        	temp_percs['livarno']=percentage_l
        	temp_percs['mushroom']=percentage_m
        	temp_percs['standard']=percentage_s

        	percs[str(label)]=temp_percs

    print(percs)
    f.close()
    return percs

def show_map(points):

"""Function used to plot the obervation sequence based on the values returned by the planning algorithms"""

	positions=['11','10','21','22','22','10','9','20','21','22']
	actions=['south','south','south','south','north','north','west','south','east']

	px=[]
	py=[]
	pz=[]
	label=[]

	fig = plt.figure()
	ax = fig.add_subplot(111,projection='3d')



	for k,v in points.items():

		px.append(float(v['x']))
		py.append(float(v['y']))
		pz.append(float(v['z']))
		label.append(k)


	print(px)
	print(py)
	print(pz)
	print(label)

	ax.grid('on')
	ax.scatter(px,py,pz)

	ax.set_xlabel('x')
	ax.set_ylabel('y')
	ax.set_zlabel('z')

	for i in range(1,len(positions)):

		x2=float(points[positions[i]]['x'])
		x1=float(points[positions[i-1]]['x'])

		y2=float(points[positions[i]]['y'])
		y1=float(points[positions[i-1]]['y'])

		z2=float(points[positions[i]]['z'])
		z1=float(points[positions[i-1]]['z'])

		q=np.sqrt((x2-x1)**2+(y2-y1)**2)
		ux=(x2-x1)/q
		uy=(y2-y1)/q
		uz=(z2-z1)/q

		for l in np.arange(0,q,.01):
			ppx=x1+l*ux
			ppy=y1+l*uy
			ppz=z1+l*uz

			if(actions[i-1]=='south'):
				color='g'
			elif actions[i-1]=='east':
				color='r'
			elif actions[i-1]=='north':
				color='y'
			else:
				color='b'


			ax.scatter(ppx,ppy,ppz,s=1,color=color)


	for i in range(0,len(px)):
		ax.text(px[i],py[i],pz[i],label[i],size='large')
		#ax.text(px[i],py[i],label[i],size='large')

	plt.grid(True)
	plt.show()


### Fetch needed data###

points_filename='/home/cptd/c/graph.txt'
elongated_filename='/home/cptd/test_results/noise_elongated.txt'
livarno_filename='/home/cptd/test_results/noise_livarno.txt'
mushroom_filename='/home/cptd/test_results/noise_mushroom.txt'
standard_filename='/home/cptd/test_results/noise_standard.txt'
bulbs=['elongated','livarno','mushroom','standard']

points=read_points(points_filename)
#points=temp



elong_percs=read_percentages(elongated_filename)
livarno_percs=read_percentages(livarno_filename)
mush_percs=read_percentages(mushroom_filename)
std_percs=read_percentages(standard_filename)

percs=[]
percs.append(elong_percs)
percs.append(livarno_percs)
percs.append(mush_percs)
percs.append(std_percs)

show_map(points)

########################