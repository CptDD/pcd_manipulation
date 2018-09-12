#!/usr/bin/python

import xml.etree.cElementTree as ET

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

	for k,v in points.items():
		if 'decision' in actions:
			tempEntry=tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='decision vp'+str(k)+' vp'+str(k)
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'


	for k,v in points.items():
		for bulb in bulbs:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='d_'+str(bulb)+' vp'+str(k)+' vp'+str(k)
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


'''def compute_total_sum(percs,point,bulbs):
	total_sum=0

	for bulb in bulbs:
		print(percs[point][bulb])

		total_sum+=float(str(percs[point][bulb]))


		#print(str(i)+' bulb :'+bulb+' '+str(percs[i][point][bulb]))
		#total_sum+=float(str(percs[i][point]))
	return total_sum'''

def compute_total_sum(percs,point,bulb):

	total_sum=0


	print(percs[0])

	print('\n\n\n')
	print(percs[1])


	for i in range(0,len(bulbs)):

		#print(percs[i][str(point)][bulb])
		total_sum+=float(str(percs[i][str(point)][bulb]))

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

				'''if bulb == 'elongated':
					total_sum=compute_total_sum(el,k,bulbs)
				if bulb == 'livarno':
					total_sum=compute_total_sum(liv,k,bulbs)

				if bulb == 'mushroom':
					total_sum=compute_total_sum(mush,k,bulbs)

				if bulb == 'standard':
					total_sum=compute_total_sum(standard,k,bulbs)'''


				norm_fact=compute_normalisation_factor(total_sum,bulbs)

				print('Total sum for :'+str(bulb)+' and point '+str(k)+' is '+str(total_sum))

				print('Normalisation factor :'+str(norm_fact))

				print('---')
				for i in range(0,len(bulbs)):
					tempEntry=ET.SubElement(parent,'Entry')
					tempInstance=ET.SubElement(tempEntry,'Instance').text=act+' vp'+str(k)+' '+bulb+' o'+bulbs[i]

					initVal=float(percs[i][k][bulb])

					print('Init val :'+str(initVal))
					tempProbTable=ET.SubElement(tempEntry,'ProbTable')
					

					if total_sum>1:
						tempProbTable.text=str(initVal/norm_fact)
					elif total_sum<1:
						tempProbTable.text=str(initVal+norm_fact)
					else:
						tempProbTable.text=str(initVal)

					#tempProbTable.text=str(initVal)
					print('Temp prob text:'+tempProbTable.text)
					#total_sum+=float(percs[i][k][bulbs[i]])

	for bulb in bulbs:
		for k,v in points.items():
			for state in bulbs:
				for obs in bulbs:
					tempEntry=ET.SubElement(parent,'Entry')
					tempInstance=ET.SubElement(tempEntry,'Instance').text='d_'+bulb+' vp'+str(k)+' '+state+' o'+obs
					tempProbTable=ET.SubElement(tempEntry,'ProbTable').text=str(float(1./len(bulbs)))


def read_points(filename):
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
					temp=line.split('\t\t')
					for i in range(1,len(temp)):
						neighbours.append(temp[i].strip())

					temp_tags['neighbours']=neighbours
		else:
			points[label]=temp_tags

	f.close()
	return points


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

				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y2-y1,color='g')

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


				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=y1-y2,color='y')

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

				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x1-x2,color='r')

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

				ax.arrow(x2,y2,x1-x2,y1-y2,head_length=x2-x1,color='b')

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

#points_filename='/home/cptd/c/graph_test.txt'
points_filename='/home/cptd/go_ws/src/bulb_scanner/graph/valid.txt'

elongated_filename='/home/cptd/go_ws/src/bulb_processor/results/multi/ee.txt'
livarno_filename='/home/cptd/go_ws/src/bulb_processor/results/multi/el.txt'

ee_filename='/home/cptd/go_ws/src/bulb_processor/results/multi/ee.txt'
el_filename='/home/cptd/go_ws/src/bulb_processor/results/multi/el.txt'

le_filename='/home/cptd/go_ws/src/bulb_processor/results/multi/le.txt'
ll_filename='/home/cptd/go_ws/src/bulb_processor/results/multi/ll.txt'

#mushroom_filename='/home/cptd/test_results/gen_mushroom.txt'
#standard_filename='/home/cptd/test_results/gen_standard.txt'

bulbs=['elongated','livarno']

points=read_points(points_filename)
#points=temp


ee_percs=read_percentages(ee_filename)
el_percs=read_percentages(el_filename)

le_percs=read_percentages(le_filename)
ll_percs=read_percentages(ll_filename)

#mush_percs=read_percentages(mushroom_filename)
#std_percs=read_percentages(standard_filename)

percs=[]
percs.append(ee_percs)
percs.append(el_percs)

percs.append(le_percs)
percs.append(ll_percs)

show_map(points)


########################


root=ET.Element('pomdpx',version='0.1',id='autogenerated')
discount=ET.SubElement(root,'Discount').text='0.95'
horizon=ET.SubElement(root,'Horizon').text='10'

variable=ET.SubElement(root,'Variable')

fullyObsVars=ET.SubElement(variable,'StateVar',vnamePrev='robot_0',vnameCurr='robot_1',fullyObs='true')
fullyObsEnums=ET.SubElement(fullyObsVars,'ValueEnum').text=add_fullyObs_vars(points)

notFullyObsVars1=ET.SubElement(variable,'StateVar',vnamePrev='state_0',vnameCurr='state_1',fullyObs='false')
notFullyObsEnums1=ET.SubElement(notFullyObsVars1,'ValueEnum').text=add_notFullyObs_vars(bulbs)

notFullyObsVars2=ET.SubElement(variable,'StateVar',vnamePrev='state_2',vnameCurr='state_3',fullyObs='false')
notFullyObsEnums2=ET.SubElement(notFullyObsVars2,'ValueEnum').text=add_notFullyObs_vars(bulbs)


obsVars1=ET.SubElement(variable,'ObsVar',vname='obs_sensor_1')
obsEnums1=ET.SubElement(obsVars1,'ValueEnum').text=add_obs_vars(bulbs)

obsVars2=ET.SubElement(variable,'ObsVar',vname='obs_sensor_2')
obsEnums2=ET.SubElement(obsVars2,'ValueEnum').text=add_obs_vars(bulbs)


actionVars=ET.SubElement(variable,'ActionVar',vname='action_robot')
actionEnums=ET.SubElement(actionVars,'ValueEnum').text=add_action_vars(bulbs)

rewardVars1=ET.SubElement(variable,'RewardVar',vname='reward_bulb_1')
rewardVars2=ET.SubElement(variable,'RewardVar',vname='reward_bulb_2')



initialBS=ET.SubElement(root,'InitialStateBelief')
fullyCondProb=ET.SubElement(initialBS,'CondProb')
fullyVar=ET.SubElement(fullyCondProb,'Var').text='robot_0'
fullyParent=ET.SubElement(fullyCondProb,'Parent').text='null'
fullyParameter=ET.SubElement(fullyCondProb,'Parameter',type='TBL')
fullyEntry=ET.SubElement(fullyParameter,'Entry')
fullyInstance=ET.SubElement(fullyEntry,'Instance').text='-'
fullyProbTable=ET.SubElement(fullyEntry,'ProbTable').text=prepare_fully_bs(points)

notFullyCondProb1=ET.SubElement(initialBS,'CondProb')
notFullyVar1=ET.SubElement(notFullyCondProb1,'Var').text='state_0'
notFullyParent1=ET.SubElement(notFullyCondProb1,'Parent').text='null'
notFullyParameter1=ET.SubElement(notFullyCondProb1,'Parameter',type='TBL')
notFullyEntry1=ET.SubElement(notFullyParameter1,'Entry')
notFullyInstance1=ET.SubElement(notFullyEntry1,'Instance').text='-'
notFullyProbTable1=ET.SubElement(notFullyEntry1,'ProbTable').text='uniform'


notFullyCondProb2=ET.SubElement(initialBS,'CondProb')
notFullyVar2=ET.SubElement(notFullyCondProb2,'Var').text='state_2'
notFullyParent2=ET.SubElement(notFullyCondProb2,'Parent').text='null'
notFullyParameter2=ET.SubElement(notFullyCondProb2,'Parameter',type='TBL')
notFullyEntry2=ET.SubElement(notFullyParameter2,'Entry')
notFullyInstance2=ET.SubElement(notFullyEntry2,'Instance').text='-'
notFullyProbTable2=ET.SubElement(notFullyEntry2,'ProbTable').text='uniform'

stateTransFunction=ET.SubElement(root,'StateTransitionFunction')
transFullyCondProb=ET.SubElement(stateTransFunction,'CondProb')
transFullyVar=ET.SubElement(transFullyCondProb,'Var').text='robot_1'
transFullyParent=ET.SubElement(transFullyCondProb,'Parent').text='action_robot robot_0'
transFullyParameter=ET.SubElement(transFullyCondProb,'Parameter',type='TBL')
prepare_fully_state_transition_entries(transFullyParameter,points,bulbs)

transNotFullyCondProb1=ET.SubElement(stateTransFunction,'CondProb')
transNotFullyVar1=ET.SubElement(transNotFullyCondProb1,'Var').text='state_1'
transNotFullyParent1=ET.SubElement(transNotFullyCondProb1,'Parent').text='action_robot state_0'
transNotFullyParameter1=ET.SubElement(transNotFullyCondProb1,'Parameter',type='TBL')
prepare_notFully_state_transition_entries(transNotFullyParameter1,bulbs)

transNotFullyCondProb2=ET.SubElement(stateTransFunction,'CondProb')
transNotFullyVar2=ET.SubElement(transNotFullyCondProb2,'Var').text='state_3'
transNotFullyParent2=ET.SubElement(transNotFullyCondProb2,'Parent').text='action_robot state_2'
transNotFullyParameter2=ET.SubElement(transNotFullyCondProb2,'Parameter',type='TBL')
prepare_notFully_state_transition_entries(transNotFullyParameter2,bulbs)





obsFunction=ET.SubElement(root,'ObsFunction')
obsCondProb1=ET.SubElement(obsFunction,'CondProb')
obsVar1=ET.SubElement(obsCondProb1,'Var').text='obs_sensor_1'
obsParent1=ET.SubElement(obsCondProb1,'Parent').text='action_robot robot_1 state_1'
obsParameter1=ET.SubElement(obsCondProb1,'Parameter',type='TBL')
prepare_obs_function_entries(obsParameter1,points,bulbs,percs[0:2])

obsCondProb2=ET.SubElement(obsFunction,'CondProb')
obsVar2=ET.SubElement(obsCondProb2,'Var').text='obs_sensor_2'
obsParent2=ET.SubElement(obsCondProb2,'Parent').text='action_robot robot_1 state_3'
obsParameter2=ET.SubElement(obsCondProb2,'Parameter',type='TBL')
prepare_obs_function_entries(obsParameter2,points,bulbs,percs[2:])


rewardFunc=ET.SubElement(root,'RewardFunction')
func1=ET.SubElement(rewardFunc,'Func')
rewardVar1=ET.SubElement(func1,'Var').text='reward_bulb_1'
rewardParent1=ET.SubElement(func1,'Parent').text='action_robot robot_0 state_0 state_1'
rewardParameter1=ET.SubElement(func1,'Parameter',type='TBL')
prepare_reward_entries(rewardParameter1,bulbs)

func2=ET.SubElement(rewardFunc,'Func')
rewardVar2=ET.SubElement(func2,'Var').text='reward_bulb_2'
rewardParent2=ET.SubElement(func2,'Parent').text='action_robot robot_0 state_2 state_3'
rewardParameter2=ET.SubElement(func2,'Parameter',type='TBL')
prepare_reward_entries(rewardParameter2,bulbs)

 

tree = ET.ElementTree(root)
tree.write("filename.xml")


