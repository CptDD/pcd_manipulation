#!/usr/bin/python

import xml.etree.cElementTree as ET


import rospy
import rospkg

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import operator

def  add_fullyObs_vars(points):

	text=''
	for k,v in points.items():
		text+='vp'+str(k)+' '
	return text.rstrip()

def add_notFullyObs_vars(bulbs):
	text=''
	for i in bulbs:
		text+=(i+' ')
	return text.rstrip()

def add_obs_vars(bulbs):
	text=''
	for i in bulbs:
		text+=('o'+i+' ')
	return text.rstrip()

def add_action_vars(bulbs):
	text=''
	for action in get_actions():
		text+=action+' '

	for bulb in bulbs:
		text+='d_'+bulb+' '
	return text.rstrip()

def get_actions():
	actions=['south','north','east','west']
	return actions


def prepare_fully_bs(points):
	text='1 '
	for i in range(1,len(points)):
		text+='0 '
	return text.rstrip()


def prepare_fully_state_transition_entries(parent,points,bulbs):


	actions=get_actions()

	for k,v in points.items():

		if 'south' in actions:
			south_neigh=get_south_neighbours(k,v['neighbours'],points)
			if(len(south_neigh)>0):
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='south vp'+str(k)+' vp'+str(south_neigh[0][0])
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'
			else:
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='south vp'+str(k)+' vp'+str(k)
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	for k,v in points.items():

		if 'north' in actions:
			north_neigh=get_north_neighbours(k,v['neighbours'],points)
			if(len(north_neigh)>0):
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='north vp'+str(k)+' vp'+str(north_neigh[0][0])
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'
			else:
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='north vp'+str(k)+' vp'+str(k)
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	for k,v in points.items():

		if 'east' in actions:
			east_neigh=get_east_neighbours(k,v['neighbours'],points)
			if(len(east_neigh)>0):
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='east vp'+str(k)+' vp'+str(east_neigh[0][0])
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'
			else:
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='east vp'+str(k)+' vp'+str(k)
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	for k,v in points.items():

		if 'west' in actions:
			west_neigh=get_west_neighbours(k,v['neighbours'],points)
			if(len(west_neigh)>0):
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='west vp'+str(k)+' vp'+str(west_neigh[0][0])
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'
			else:
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='west vp'+str(k)+' vp'+str(k)
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'




		if 'up' in actions:
			up_neigh=get_up(k,points)
			if(len(up_neigh)>0):
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='up vp'+str(k)+' vp'+str(up_neigh[0])
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

			xtr_up=get_extreme_up(points)
			tempEntry=tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='up vp'+str(xtr_up[0])+' vp'+str(xtr_up[0])
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'


		if 'down' in actions:
			down_neigh=get_west(k,points)
			if(len(down_neigh)>0):
				tempEntry=tempEntry=ET.SubElement(parent,'Entry')
				tempInstance=ET.SubElement(tempEntry,'Instance').text='down vp'+str(k)+' vp'+str(down_neigh[0])
				tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

			xtr_down=get_extreme_down(points)
			tempEntry=tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='down vp'+str(xtr_down[0])+' vp'+str(xtr_down[0])
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	for k,v in points.items():
		for bulb in bulbs:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='d_'+str(bulb)+' vp'+str(k)+' vp'+str(k)
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	'''if 'south' in actions:
		xtr_south=get_extreme_south(points)
		tempEntry=tempEntry=ET.SubElement(parent,'Entry')
		tempInstance=ET.SubElement(tempEntry,'Instance').text='south vp'+str(xtr_south[0])+' vp'+str(xtr_south[0])
		tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	if 'north' in actions:
		xtr_north=get_extreme_north(points)
		tempEntry=tempEntry=ET.SubElement(parent,'Entry')
		tempInstance=ET.SubElement(tempEntry,'Instance').text='north vp'+str(xtr_north[0])+' vp'+str(xtr_north[0])
		tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	if 'east' in actions:
		xtr_east=get_extreme_east(points)
		tempEntry=tempEntry=ET.SubElement(parent,'Entry')
		tempInstance=ET.SubElement(tempEntry,'Instance').text='east vp'+str(xtr_east[0])+' vp'+str(xtr_east[0])
		tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	if 'west' in actions:
		xtr_west=get_extreme_west(points)
		tempEntry=tempEntry=ET.SubElement(parent,'Entry')
		tempInstance=ET.SubElement(tempEntry,'Instance').text='west vp'+str(xtr_west[0])+' vp'+str(xtr_west[0])
		tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'''




def prepare_notFully_state_transition_entries(parent,bulbs):
	for action in get_actions():
		for i in bulbs:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text=action+' '+i+' '+i
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

	for bulb in bulbs:
		for temp_bulb in bulbs:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='d_'+bulb+' '+temp_bulb+' '+temp_bulb
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'


def prepare_reward_entries(parent,bulbs):
	actions=get_actions()

	for act in actions:
		tempEntry=ET.SubElement(parent,'Entry')
		tempInstance=ET.SubElement(tempEntry,'Instance').text=act+' * * *'
		valueTable=ET.SubElement(tempEntry,'ValueTable').text='-1'

	for bulb in bulbs:
		for state in bulbs:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='d_'+bulb+' * '+state+' *'
			valueTable=ET.SubElement(tempEntry,'ValueTable')

			if bulb==state:
				valueTable.text='10'
			else:
				valueTable.text='-10'

def compute_total_sum(percs,point,bulb):
	total_sum=0

	for i in range(0,len(percs)):
		print(percs[i][point][bulb])
		total_sum+=float(str(percs[i][point][bulb]))

	#print(total_sum)

	return total_sum

def compute_normalisation_factor(total_sum,bulbs):
	if(total_sum>1):
		return total_sum
	else:
		norm_fact=(1-total_sum)/len(bulbs)
		return norm_fact


def is_south(target,coords):

	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['y'])<y):
		return True
	else:
		return False

def is_north(target,coords):

	
	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['y'])>y):
		return True
	else:
		return False

def is_west(target,coords):

	
	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['x'])<x):
		return True
	else:
		return False

def is_east(target,coords):

	
	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['x'])>x):
		return True
	else:
		return False


def is_up(target,coords):
	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['z'])>z):
		return True
	else:
		return False


def is_down(target,coords):

	x=float(target['x'])
	y=float(target['y'])
	z=float(target['z'])

	if(float(coords['z'])<z):
		return True
	else:
		return False

def get_extreme_south(points):

	print('---Extreme south---')

	keys=points.keys()

	extr_key=keys[0]
	extr_coords=points[extr_key]

	for k,v in points.items():
		if k!=extr_key:
			if(is_south(extr_coords,v)):
				extr_coords=v
				extr_key=k

	return extr_key,extr_coords


def get_extreme_north(points):

	print('---Extreme north---')

	keys=points.keys()

	extr_key=keys[0]
	extr_coords=points[extr_key]

	for k,v in points.items():
		if k!=extr_key:
			if(is_north(extr_coords,v)):
				extr_coords=v
				extr_key=k

	return extr_key,extr_coords


def get_extreme_east(points):

	print('---Extreme east---')

	keys=points.keys()

	extr_key=keys[0]
	extr_coords=points[extr_key]

	for k,v in points.items():
		if k!=extr_key:
			if(is_east(extr_coords,v)):
				extr_coords=v
				extr_key=k

	return extr_key,extr_coords

def get_extreme_west(points):

	print('---Extreme west---')

	keys=points.keys()

	extr_key=keys[0]
	extr_coords=points[extr_key]

	for k,v in points.items():
		if k!=extr_key:
			if(is_west(extr_coords,v)):
				extr_coords=v
				extr_key=k

	return extr_key,extr_coords


def get_extreme_up(points):

	print('---Extreme up---')

	keys=points.keys()

	extr_key=keys[0]
	extr_coords=points[extr_key]

	for k,v in points.items():
		if k!=extr_key:
			if(is_up(extr_coords,v)):
				extr_coords=v
				extr_key=k

	return extr_key,extr_coords


def get_extreme_down(points):

	print('---Extreme down---')

	keys=points.keys()

	extr_key=keys[0]
	extr_coords=points[extr_key]

	for k,v in points.items():
		if k!=extr_key:
			if(is_down(extr_coords,v)):
				extr_coords=v
				extr_key=k

	return extr_key,extr_coords




def get_south(target,points):

	south_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_south(points[target],v)):
				south_neigh.append(k)

	return south_neigh



def get_north(target,points):

	north_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_north(points[target],v)):
				north_neigh.append(k)
	return north_neigh

def get_east(target,points):

	east_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_east(points[target],v)):
				east_neigh.append(k)
	return east_neigh


def get_west(target,points):

	west_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_west(points[target],v)):
				west_neigh.append(k)
	return west_neigh

def get_up(target,points):

	up_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_up(points[target],v)):
				up_neigh.append(k)
	return up_neigh

def get_down(target,points):

	down_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_down(points[target],v)):
				down_neigh.append(k)
	return down_neigh


def get_closest_south(target,neighbours,points):


	south_neighs=dict()
	for i in neighbours:

		if(is_south(points[target],points[i])):
	
			if(float(points[target]['y'])<=0):
				diff=abs(float(points[target]['y']))-abs(float(points[i]['y']))
			else:
				diff=abs(float(points[i]['y']))-abs(float(points[target]['y']))

			if(diff<-0.044):
				south_neighs[i]=diff

	sorted_list= sorted(south_neighs.items(), key=operator.itemgetter(0))

	return sorted_list
	

def get_closest_north(target,neighbours,points):

	north_neighs=dict()
	for i in neighbours:

		if(is_north(points[target],points[i])):

			if(float(points[target]['y'])<0):
				diff=abs(float(points[target]['y']))-abs(float(points[i]['y']))
			else:
				diff=abs(float(points[i]['y']))-abs(float(points[target]['y']))

			if(diff>0.044):
				north_neighs[i]=diff


	sorted_list=sorted(north_neighs.items(),key=operator.itemgetter(0))

	return sorted_list

def get_closest_east(target,neighbours,points):

	east_neighs=dict()

	for i in neighbours:

		if(is_east(points[target],points[i])):

			if(float(points[target]['x'])>0):
				diff=abs(float(points[target]['x']))-abs(float(points[i]['x']))
				if diff<-0.05:
					east_neighs[i]=diff
			else:
				diff=abs(float(points[i]['x']))-abs(float(points[target]['x']))
				if diff>0.05:
					east_neighs[i]=diff


	sorted_list=sorted(east_neighs.items(),key=operator.itemgetter(0),reverse=True)

	return sorted_list

def get_closest_west(target,neighbours,points):

	west_neighs=dict()

	for i in neighbours:

		if(is_west(points[target],points[i])):


			if(float(points[target]['x'])>0 and float(points[i]['x'])>0):
				diff=abs(float(points[target]['x']))-abs(float(points[i]['x']))

				if diff>0.04:
					west_neighs[i]=diff


			elif(float(points[target]['x'])<0 and float(points[i]['x'])<0):
				diff=abs(float(points[i]['x']))-abs(float(points[target]['x']))

				if diff>0.04:
					west_neighs[i]=diff

			else:
				diff=abs(float(points[target]['x']))-abs(float(points[i]['x']))

				if diff<-0.05:
					west_neighs[i]=diff


	sorted_list=sorted(west_neighs.items(),key=operator.itemgetter(0))

	return sorted_list



def get_south_neighbours(target,neighbours,points):

	south_neigh=[]
	south_neigh=get_closest_south(target,neighbours,points)
		#if(is_south(points[target],points[i])):
		#	south_neigh.append(i)


	return south_neigh

def get_north_neighbours(target,neighbours,points):

	north_neigh=[]

	north_neigh=get_closest_north(target,neighbours,points)

	#for i in neighbours:
	#	if(is_north(points[target],points[i])):
	#		north_neigh.append(i)


	return north_neigh

def get_east_neighbours(target,neighbours,points):

	east_neigh=[]
	east_neigh=get_closest_east(target,neighbours,points)


	#for i in neighbours:
	#	if(is_east(points[target],points[i])):
	#		east_neigh.append(i)

	return east_neigh

def get_west_neighbours(target,neighbours,points):

	west_neigh=[]
	west_neigh=get_closest_west(target,neighbours,points)

	#for i in neighbours:
	#	if(is_west(points[target],points[i])):
	#		west_neigh.append(i)

	return west_neigh


def  prepare_obs_function_entries(parent,points,bulbs,percs):

	if 'elongated' in bulbs:
		el=percs[0]

	if 'livarno' in bulbs:
		liv=percs[1]

	if 'mushroom' in bulbs:
		mush=percs[2]

	if 'standard' in bulbs:
		standard=percs[3]


	#el=percs[0]
	#liv=percs[1]
	#mush=percs[2]
	#standard=percs[3]

	actions=get_actions()

	for act in actions:
		print(act)
		for k,v in points.items():
			for bulb in bulbs:
				total_sum=compute_total_sum(percs,k,bulb)
				norm_fact=compute_normalisation_factor(total_sum,bulbs)

				print('Total sum for :'+str(bulb)+' '+str(total_sum)+' normalisation factor is :'+str(norm_fact))
				print('---')

				for i in range(0,len(bulbs)):
					tempEntry=ET.SubElement(parent,'Entry')
					tempInstance=ET.SubElement(tempEntry,'Instance').text=act+' vp'+str(k)+' '+bulb+' o'+bulbs[i]

					initVal=float(percs[i][k][bulb])
					tempProbTable=ET.SubElement(tempEntry,'ProbTable')
			
					if total_sum>1:
						tempProbTable.text=str(initVal/norm_fact)
					elif total_sum<1:
						tempProbTable.text=str(initVal+norm_fact)
					else:
						tempProbTable.text=str(initVal)

					#print(tempProbTable.text)
					#total_sum+=float(percs[i][k][bulbs[i]])'''


	for bulb in bulbs:
		for k,v in points.items():
			for state in bulbs:
				for obs in bulbs:
					tempEntry=ET.SubElement(parent,'Entry')
					tempInstance=ET.SubElement(tempEntry,'Instance').text='d_'+bulb+' vp'+str(k)+' '+state+' o'+obs
					tempProbTable=ET.SubElement(tempEntry,'ProbTable').text=str(float(1./len(bulbs)))

def read_percentages(filename):
    f=open(filename,'r')
    percs=dict()

    i=0

    for line in f:
        if '---' not in line:
            if 'Label:' in line:
                label=line.split('\t')[1].strip().replace('p','').rstrip()
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



def read_valid_points(filename):
	f=open(filename,'r')

	points=dict()

	for line in f:
		if '---' not in line:
			if '___' not in line:
				if 'Original' in line:
					temp_tags=dict()
					label=line.split(' ')[1].rstrip().replace('p','')

				if 'Position' in line:
					line=line.replace('Position :','').rstrip()
					pos=line.split('\t')
					temp_tags['x']=pos[0]
					temp_tags['y']=pos[1]
					temp_tags['z']=pos[2]

				if 'Orientation' in line:
					line=line.replace('Orientation :','').rstrip()
					line=line.split('\t')
					temp_tags['xo']=line[0]
					temp_tags['yo']=line[1]
					temp_tags['zo']=line[2]
					temp_tags['wo']=line[3]

				if 'Neighbours' in line:
					neighbours=[]
					temp=line.split('\t')
					for i in range(1,len(temp)):
						neighbours.append(temp[i].strip())

					temp_tags['neighbours']=neighbours
		else:
			points[label]=temp_tags

	f.close()
	return points


def show_map(points):

	actions=get_actions()
	px=[]
	py=[]
	pz=[]
	label=[]

	fig = plt.figure()
	ax = fig.add_subplot(111)

	for k,v in points.items():

		px.append(float(v['x']))
		py.append(float(v['y']))
		pz.append(float(v['z']))
		label.append(k)


		print('---Neighbours for :'+str(k)+'---')

		if 'south' in actions:

			south_neigh=get_south_neighbours(k,v['neighbours'],points)

			if(len(south_neigh)>0):
				print('South :'+str(south_neigh[0][0]))

				x2=float(points[south_neigh[0][0]]['x'])
				x1=float(v['x'])
				y2=float(points[south_neigh[0][0]]['y'])
				y1=float(v['y'])

				q=np.sqrt((x2-x1)**2+(y2-y1)**2)

				ux=(x2-x1)/q
				uy=(y2-y1)/q

				print(y2-y1)

				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=0,color='g')
				#ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y2-y1,color='g')

				'''if x1>0 and y1>0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y2-y1,color='g')
				elif x1<0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y2-y1,color='g')

				elif x1>0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y2-y1,color='g')
				else:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y2-y1,color='g')'''



				#else:
					#ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x2-x1,color='g')


				'''for l in np.arange(0,q,.01):
					ppx=x1+l*ux
					ppy=y1+l*uy
					ax.scatter(ppx,ppy,s=1,color='g')'''



		if 'north' in actions:
			north_neigh=get_north_neighbours(k,v['neighbours'],points)
			if(len(north_neigh)>0):
				print('North :'+str(north_neigh[0][0]))


				x2=float(points[north_neigh[0][0]]['x'])
				x1=float(v['x'])
				y2=float(points[north_neigh[0][0]]['y'])
				y1=float(v['y'])

				q=np.sqrt((x2-x1)**2+(y2-y1)**2)

				ux=(x2-x1)/q
				uy=(y2-y1)/q


				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=0,color='y')

				'''if x1>0 and y1>0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y1-y2,color='y')
				elif x1<0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y1-y2,color='y')

				elif x1>0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y1-y2,color='y')

				else:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y1-y2,color='y')'''

				'''for l in np.arange(0,q,.01):
					ppx=x1+l*ux
					ppy=y1+l*uy
					ax.scatter(ppx,ppy,s=1,color='y')'''



		if 'east' in actions:
			east_neigh=get_east_neighbours(k,v['neighbours'],points)
			if(len(east_neigh)>0):
				print('East :'+str(east_neigh[0][0]))

				x2=float(points[east_neigh[0][0]]['x'])
				x1=float(v['x'])
				y2=float(points[east_neigh[0][0]]['y'])
				y1=float(v['y'])

				q=np.sqrt((x2-x1)**2+(y2-y1)**2)

				ux=(x2-x1)/q
				uy=(y2-y1)/q

				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=0,color='r')

				'''if x1>0 and y1>0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x1-x2,color='r')
				elif x1<0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x1-x2,color='r')

				elif x1>0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x2-x1,color='r')
				else:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x2-x1,color='r')'''

				'''for l in np.arange(0,q,.01):
					ppx=x1+l*ux
					ppy=y1+l*uy
					ax.scatter(ppx,ppy,s=1,color='r')'''





		if 'west' in actions:
			west_neigh=get_west_neighbours(k,v['neighbours'],points)
			if(len(west_neigh)>0):
				print('West :'+str(west_neigh[0][0]))

				x2=float(points[west_neigh[0][0]]['x'])
				x1=float(v['x'])
				y2=float(points[west_neigh[0][0]]['y'])
				y1=float(v['y'])

				q=np.sqrt((x2-x1)**2+(y2-y1)**2)

				ux=(x2-x1)/q
				uy=(y2-y1)/q

				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=0,color='b')

				'''if x1>0 and y1>0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x1-x2,color='b')
				elif x1<0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x1-x2,color='b')

				elif x1>0 and y1<0:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x2-x1,color='b')
				else:
					ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x2-x1,color='b')'''

				'''for l in np.arange(0,q,.01):
					ppx=x1+l*ux
					ppy=y1+l*uy
					ax.scatter(ppx,ppy,s=1,color='b')'''


	xtr_south=get_extreme_south(points)
	print('Extreme south :'+str(xtr_south[0]))

	xtr_north=get_extreme_north(points)
	print('Extreme north :'+str(xtr_north[0]))

	xtr_east=get_extreme_east(points)
	print('Extreme east :'+str(xtr_east[0]))

	xtr_west=get_extreme_west(points)
	print('Extreme west :'+str(xtr_west[0]))

	print(px)
	print(py)
	print(label)


	ax.grid('on')
	ax.scatter(px,py)

	ax.set_xlabel('x')
	ax.set_ylabel('y')
	#ax.set_zlabel('z')



	for i in range(0,len(px)):
		#ax.text(px[i],py[i],pz[i],label[i],size='large')
		ax.text(px[i],py[i],label[i],size='large')

	ax.set_title('Movement map')
	ax.set_xlabel('x')
	ax.set_ylabel('y')



	plt.legend(loc='best')
	plt.show()


### Fetch needed data###
########################


def main():
	rospy.init_node('bulb_pomdpx_builder')

	rospack=rospkg.RosPack()

	path=rospack.get_path('bulb_scanner')
	
	points_filename=path+'/graph/valid_elongated.txt'
	path=rospack.get_path('bulb_processor')

	elongated_filename=path+'/results/elongated.txt'
	livarno_filename=path+'/results/livarno.txt'
	mushroom_filename=path+'/results/mushroom.txt'
	standard_filename=path+'/results/standard.txt'

	bulbs=['elongated','livarno']

	points=read_valid_points(points_filename)
	elong_percs=read_percentages(elongated_filename)
	livarno_percs=read_percentages(livarno_filename)
	mush_percs=read_percentages(mushroom_filename)
	std_percs=read_percentages(standard_filename)

	percs=[]

	if 'elongated' in bulbs:
		percs.append(elong_percs)
	if 'livarno' in bulbs:
		percs.append(livarno_percs)

	if 'mushroom' in bulbs:
		percs.append(mush_percs)

	if 'standard' in bulbs:
		percs.append(std_percs)

	show_map(points)

	root=ET.Element('pomdpx',version='0.1',id='autogenerated')
	discount=ET.SubElement(root,'Discount').text='0.95'
	horizon=ET.SubElement(root,'Horizon').text='10'

	variable=ET.SubElement(root,'Variable')
	fullyObsVars=ET.SubElement(variable,'StateVar',vnamePrev='robot_0',vnameCurr='robot_1',fullyObs='true')
	fullyObsEnums=ET.SubElement(fullyObsVars,'ValueEnum').text=add_fullyObs_vars(points)

	notFullyObsVars=ET.SubElement(variable,'StateVar',vnamePrev='state_0',vnameCurr='state_1',fullyObs='false')
	notFullyObsEnums=ET.SubElement(notFullyObsVars,'ValueEnum').text=add_notFullyObs_vars(bulbs)

	obsVars=ET.SubElement(variable,'ObsVar',vname='obs_sensor')
	obsEnums=ET.SubElement(obsVars,'ValueEnum').text=add_obs_vars(bulbs)

	actionVars=ET.SubElement(variable,'ActionVar',vname='action_robot')
	actionEnums=ET.SubElement(actionVars,'ValueEnum').text=add_action_vars(bulbs)

	rewardVars=ET.SubElement(variable,'RewardVar',vname='reward_bulb')

	initialBS=ET.SubElement(root,'InitialStateBelief')
	fullyCondProb=ET.SubElement(initialBS,'CondProb')
	fullyVar=ET.SubElement(fullyCondProb,'Var').text='robot_0'
	fullyParent=ET.SubElement(fullyCondProb,'Parent').text='null'
	fullyParameter=ET.SubElement(fullyCondProb,'Parameter',type='TBL')
	fullyEntry=ET.SubElement(fullyParameter,'Entry')
	fullyInstance=ET.SubElement(fullyEntry,'Instance').text='-'
	fullyProbTable=ET.SubElement(fullyEntry,'ProbTable').text=prepare_fully_bs(points)

	notFullyCondProb=ET.SubElement(initialBS,'CondProb')
	notFullyVar=ET.SubElement(notFullyCondProb,'Var').text='state_0'
	notFullyParent=ET.SubElement(notFullyCondProb,'Parent').text='null'
	notFullyParameter=ET.SubElement(notFullyCondProb,'Parameter',type='TBL')
	notFullyEntry=ET.SubElement(notFullyParameter,'Entry')
	notFullyInstance=ET.SubElement(notFullyEntry,'Instance').text='-'
	notFullyProbTable=ET.SubElement(notFullyEntry,'ProbTable').text='uniform'

	stateTransFunction=ET.SubElement(root,'StateTransitionFunction')
	transFullyCondProb=ET.SubElement(stateTransFunction,'CondProb')
	transFullyVar=ET.SubElement(transFullyCondProb,'Var').text='robot_1'
	transFullyParent=ET.SubElement(transFullyCondProb,'Parent').text='action_robot robot_0'
	transFullyParameter=ET.SubElement(transFullyCondProb,'Parameter',type='TBL')
	prepare_fully_state_transition_entries(transFullyParameter,points,bulbs)

	transNotFullyCondProb=ET.SubElement(stateTransFunction,'CondProb')
	transNotFullyVar=ET.SubElement(transNotFullyCondProb,'Var').text='state_1'
	transNotFullyParent=ET.SubElement(transNotFullyCondProb,'Parent').text='action_robot state_0'
	transNotFullyParameter=ET.SubElement(transNotFullyCondProb,'Parameter',type='TBL')
	prepare_notFully_state_transition_entries(transNotFullyParameter,bulbs)

	obsFunction=ET.SubElement(root,'ObsFunction')
	obsCondProb=ET.SubElement(obsFunction,'CondProb')
	obsVar=ET.SubElement(obsCondProb,'Var').text='obs_sensor'
	obsParent=ET.SubElement(obsCondProb,'Parent').text='action_robot robot_1 state_1'
	obsParameter=ET.SubElement(obsCondProb,'Parameter',type='TBL')
	prepare_obs_function_entries(obsParameter,points,bulbs,percs)

	rewardFunc=ET.SubElement(root,'RewardFunction')
	func=ET.SubElement(rewardFunc,'Func')
	rewardVar=ET.SubElement(func,'Var').text='reward_bulb'
	rewardParent=ET.SubElement(func,'Parent').text='action_robot robot_0 state_0 state_1'
	rewardParameter=ET.SubElement(func,'Parameter',type='TBL')
	prepare_reward_entries(rewardParameter,bulbs)

	tree = ET.ElementTree(root)

	path=rospack.get_path('bulb_pomdpx_builder')
	pomdpx_file=path+'/pomdpx/generated_pomdpx.pomdpx'

	tree.write(pomdpx_file)




if __name__ == '__main__':
	main()


