#!/usr/bin/env python


import numpy as np  

def read_results(filename):
	f=open(filename,'r')

	results=dict()

	ok=False

	count=0
	for line in f:

	
		if '###' in line:
			actions=[]
			positions=[]
			ee_state=[]
			el_state=[]
			le_state=[]
			ll_state=[]
			count+=1

		if '- Action' in line:
			action=line.replace('- Action =','').split(':')[1].rstrip()
			actions.append(action)

		if '- State' in line:
			line=line.replace('- State =','').replace('[','').replace(']','').rstrip().lstrip().replace(',','',2)
			line=line.split(' ')
			ustate_1=line[1].replace('state_1:','')
			ustate_3=line[2].replace('state_3:','')
			

		if 'Post' in line:
			ok=True


		if 'robot_1' in line and ok==True:
			line=line.replace(',','',2).replace('[','').replace(']','').replace('=',' ').lstrip().rstrip()
			line=line.split(' ')
			position=line[0].replace('robot_1:vp','')
			state_1=line[1].replace('state_1:','')
			state_3=line[2].replace('state_3:','')
			value=line[-1]

			if state_1 =='elongated' and state_3 =='elongated':
				ee_state.append(value)

			if state_1 == 'elongated' and state_3 == 'livarno':
				el_state.append(value)

			if state_1 == 'livarno' and state_3 == 'elongated':
				le_state.append(value)

			if state_1 == 'livarno' and state_3 == 'livarno':
				ll_state.append(value)

			
			

		if '- Reward' in line:
			ok=False

		if 'Simulation terminated' in line:
			temp_values=dict()
			temp_values['ustate1']=ustate_1
			temp_values['ustate3']=ustate_3
			temp_values['positions']=positions
			temp_values['actions']=actions
			temp_values['ee']=ee_state
			temp_values['el']=el_state
			temp_values['le']=le_state
			temp_values['ll']=ll_state
			results[count]=temp_values

	return results

def show_results(results):


	for k,v in results.items():
		ok=True
		for i in range(0,len(v['actions'])):

			if v['actions'][i]=='decision' and ok==True:
				ok=False
				print('---------------------------------------------------------------')
				print('Number of steps \t\t'+str(i+1))
				print('Under state 1:'+v['ustate1']+'\t'+'Under state 2:'+v['ustate3'])
				print('State 1:elongated State 2:elongated\t\t'+v['ee'][i])
				print('State 1:elongated State 2:livarno\t\t'+v['el'][i])
				print('State 1:livarno State 2:elongated\t\t'+v['le'][i])
				print('State 1:livarno State 2:livarno\t\t\t'+v['ll'][i])
				
	print('---------------------------------------------------------------')

############################
filename='/home/cptd/dd/despot/examples/pomdpx_models/res_ext.txt'

results=read_results(filename)
show_results(results)


