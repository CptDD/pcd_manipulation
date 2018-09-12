#!/usr/bin/env python


import numpy as np  
import glob
import matplotlib.pyplot as plt

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

		if '- Initial State' in line:
			ok=True

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

		if 'Chosen' in line:
			ok=False

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

def get_files(path):
	return glob.glob(str(path+'/*.txt'))


def show_measurements(results):

	for k,v in results.items():
		for i in v['ee']:
			print(i)

def process_files(files):

	elo_elo=[]
	elo_liv=[]
	liv_elo=[]
	liv_liv=[]
	elongated=[]
	livarno=[]

	decisions=[]

	bulbs_mat=[]

	first=True

	count=0
	for file in files:
		results=read_results(file)

		count+=1
		temp_res=dict()
		for k,v in results.items():

			temp_dec=[]
			

			for act in v['actions']:
				if act=='decision':
					temp_dec.append(1)
				else:
					temp_dec.append(0)

			decisions.append(temp_dec)


			for i in range(0,len(v['ee'])):
				if first:
					elo_elo.append(float(v['ee'][i]))
				else:
					elo_elo[i]+=float(v['ee'][i])


			for i in range(0,len(v['el'])):
				if first:
					elo_liv.append(float(v['el'][i]))
				else:
					elo_liv[i]+=float(v['el'][i])


			for i in range(0,len(v['le'])):
				if first:
					liv_elo.append(float(v['le'][i]))
				else:
					liv_elo[i]+=float(v['le'][i])


			for i in range(0,len(v['ll'])):
				if first:
					liv_liv.append(float(v['ll'][i]))
				else:
					liv_liv[i]+=float(v['ll'][i])


			t_e=[]
			t_l=[]

			for i in range(0,len(elo_elo)):
				t_e.append(elo_elo[i]+elo_liv[i])
				t_l.append(liv_elo[i]+liv_liv[i])

			temp_res['ee']=np.divide(elo_elo,len(files))
			temp_res['el']=np.divide(elo_liv,len(files))
			temp_res['le']=np.divide(liv_elo,len(files))
			temp_res['ll']=np.divide(liv_liv,len(files))			
			temp_res['elongated']=np.divide(t_e,count)
			temp_res['livarno']=np.divide(t_l,count)


		first=False
		bulbs_mat.append(temp_res)





	norm_factor=len(files)
	ee=np.divide(elo_elo,norm_factor)
	el=np.divide(elo_liv,norm_factor)	
	le=np.divide(liv_elo,norm_factor)
	ll=np.divide(liv_liv,norm_factor)

	for i in range(0,len(ee)):
		elongated.append(ee[i]+el[i])
		livarno.append(le[i]+ll[i])

	return ee,el,le,ll,elongated,livarno,bulbs_mat,decisions


############################
path='/home/cptd/dd/despot/examples/pomdpx_models/results_3/'
fig_path='/home/cptd/c/'

files=get_files(path)
ee,el,le,ll,elongated,livarno,bulbs_mat,decs=process_files(files)


plt.style.use('ggplot')

fig=plt.figure()
ax1 = fig.add_subplot(2,1,1)
ax1.plot(ee,marker=r'o',color=u'red',linestyle='--',label='Elongated Elongated')
ax1.plot(el,marker=r'o',color=u'blue',linestyle='--',label='Elongated Livarno')
ax1.plot(le,marker=r'o',color=u'yellow',linestyle='--',label='Livarno Elongated')
ax1.plot(ll,marker=r'o',color=u'green',linestyle='--',label='Livarno Livarno')
ax1.xaxis.set_ticks_position('bottom')
ax1.yaxis.set_ticks_position('left')
ax1.set_title('Belief state evolution')
plt.legend(loc='best')
	
ax2=fig.add_subplot(2,1,2)
ax2.plot(elongated,marker=r'o',color=u'red',linestyle='--',label='Elongated')
ax2.plot(livarno,marker=r'o',color=u'black',linestyle='--',label='Livarno')
ax2.xaxis.set_ticks_position('bottom')
ax2.yaxis.set_ticks_position('left')
ax2.set_title('Extracted belief state evolution')
plt.legend(loc='best')


for i in range(0,len(bulbs_mat)):
	temp_fig=plt.figure()
	ax=temp_fig.add_subplot(1,1,1)
	ax.plot(bulbs_mat[i]['elongated'],marker=r'o',color=u'red',linestyle='--',label='Elongated')
	ax.plot(bulbs_mat[i]['livarno'],marker=r'o',color=u'blue',linestyle='--',label='Livarno')
	ax.xaxis.set_ticks_position('bottom')
	ax.yaxis.set_ticks_position('left')
	ax.set_title('Belief state evolution step '+str(i))
	plt.legend(loc='best')

	for j in range(0,len(decs[i])):
		if decs[i][j]==1:
			ax.text(j+1,0.5,'decision')

	fig_name=fig_path+str(i)+'.png'
	temp_fig.savefig(fig_name, dpi=fig.dpi)


plt.show()

