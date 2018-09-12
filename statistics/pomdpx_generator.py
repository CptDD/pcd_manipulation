#!/usr/bin/python

import xml.etree.cElementTree as ET

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

def add_action_vars(points):
	text=''
	for k,v in points.items():
		text+='avp'+str(k)+' '
	return text.rstrip()

def get_actions(points):
	actions=[]
	for k,v in points.items():
		actions.append('avp'+str(k))
	return actions

def prepare_fully_bs(points):
	text='1 '
	for i in range(1,len(points)):
		text+='0 '
	return text.rstrip()

def prepare_fully_state_transition_entries(parent,points):

	for k,v in points.items():

		if(len(points[k]['neighbours'])>0):
			prob=1./(len(points[k]['neighbours'])+1) #+1 for the point itself
		else:
			prob=1

			print('Fully state probability: '+str(prob))

		tempEntry=ET.SubElement(parent,'Entry')
		tempInstance=ET.SubElement(tempEntry,'Instance').text='avp'+str(k)+' vp'+str(k)+' vp'+str(k)
		#tempProbTable=ET.SubElement(tempEntry,'ProbTable').text=str(prob)
		tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'

		for i in points[k]['neighbours']:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='avp'+str(k)+' vp'+str(i)+' vp'+str(k)
			#tempProbTable=ET.SubElement(tempEntry,'ProbTable').text=str(prob)
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'


def prepare_notFully_state_transition_entries(parent,points,bulbs):
	for k,v in points.items():
		for i in bulbs:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='avp'+str(k)+' '+i+' '+i
			tempProbTable=ET.SubElement(tempEntry,'ProbTable').text='1'


def prepare_reward_entries(parent,bulbs):
	for bulb in bulbs:
		for b in bulbs:
			tempEntry=ET.SubElement(parent,'Entry')
			tempInstance=ET.SubElement(tempEntry,'Instance').text='* * '+bulb+' '+b
			valueTable=ET.SubElement(tempEntry,'ValueTable')

			if bulb != b:
				valueTable.text='-50'
			else:
				valueTable.text='50'

def compute_total_sum(percs,point,bulb):
	total_sum=0

	for i in range(0,len(percs)):
		total_sum+=float(percs[i][point][bulb])

	print(total_sum)

	return total_sum

def compute_normalisation_factor(total_sum,bulbs):
	if(total_sum>1):
		return total_sum
	else:
		norm_fact=(1-total_sum)/len(bulbs)
		return norm_fact


def is_south(target,coords):

	x=target['x']
	y=target['y']
	z=target['z']

	if(coords['y']<y):
		return True
	else:
		return False

def is_north(target,coords):

	x=target['x']
	y=target['y']
	z=target['z']

	if(coords['y']>y):
		return True
	else:
		return False

def is_west(target,coords):

	x=target['x']
	y=target['y']
	z=target['z']

	if(coords['x']<x):
		return True
	else:
		return False

def is_east(target,coords):

	x=target['x']
	y=target['y']
	z=target['z']

	if(coords['x']>x):
		return True
	else:
		return False


def is_up(target,coords):
	x=target['x']
	y=target['y']
	z=target['z']

	if(coords['z']>z):
		return True
	else:
		return False


def is_down(target,coords):

	x=target['x']
	y=target['y']
	z=target['z']

	if(coords['z']<z):
		return True
	else:
		return False



def get_south(target,points):

	south_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_south(target,v)):
				south_neigh.append(k)

	return south_neigh

def get_north(target,points):

	north_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_north(target,v)):
				north_neigh.append(k)
	return north_neigh

def get_east(target,points):

	east_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_east(target,v)):
				east_neigh.append(k)
	return east_neigh


def get_west(target,points):

	west_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_west(target,v)):
				west_neigh.append(k)
	return west_neigh

def get_up(target,points):

	up_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_up(target,v)):
				up_neigh.append(k)
	return up_neigh

def get_down(target,points):

	down_neigh=[]
	for k,v in points.items():
		if target!=k:
			if(is_down(target,v)):
				down_neigh.append(k)
	return down_neigh


def  prepare_obs_function_entries(parent,points,bulbs,percs):

	el=percs[0]
	liv=percs[1]
	mush=percs[2]
	standard=percs[3]

	actions=get_actions(points)

	for act in actions:
		print(act)
		for k,v in points.items():
			for bulb in bulbs:
				total_sum=compute_total_sum(percs,k,bulb)
				norm_fact=compute_normalisation_factor(total_sum,bulbs)

				print(norm_fact)

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
					#total_sum+=float(percs[i][k][bulbs[i]])


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


### Fetch needed data###

points_filename='/home/cptd/c/graph_test.txt'
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

########################


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
actionEnums=ET.SubElement(actionVars,'ValueEnum').text=add_action_vars(points)

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
#transFullyEntry=ET.SubElement(transFullyParameter,'Entry')
prepare_fully_state_transition_entries(transFullyParameter,points)

transNotFullyCondProb=ET.SubElement(stateTransFunction,'CondProb')
transNotFullyVar=ET.SubElement(transNotFullyCondProb,'Var').text='state_1'
transNotFullyParent=ET.SubElement(transNotFullyCondProb,'Parent').text='action_robot state_1'
transNotFullyParameter=ET.SubElement(transNotFullyCondProb,'Parameter',type='TBL')
prepare_notFully_state_transition_entries(transNotFullyParameter,points,bulbs)


#transFullyInstance=ET.SubElement(transFullyEntry,'Instance').text
#transFullyProbTable=ET.SubElement(transFullyEntry,'ProbTable').text='1'''


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
tree.write("filename.xml")


