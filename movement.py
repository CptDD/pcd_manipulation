#!/usr/bin/python

from __future__ import print_function

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from matplotlib import cm
from scipy.stats import multivariate_normal
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.interpolate import griddata
from scipy.interpolate import *



import sys



'''
'''

class MovementPlotter:

	def __init__(self,point_filename,elo_perc_file,liv_perc_filename,mush_perc_filename,std_perc_filename):
		self.points_filename=point_filename
		self.elongated_filename=elo_perc_file
		self.livarno_filename=liv_perc_filename
		self.mushroom_filename=mush_perc_filename
		self.standard_filename=std_perc_filename
		self.percs=[]

		self.read_points(self.points_filename)
		self.read_percentages()
		self.reassigned=False

	def show(self,reassigned=False):

		if(reassigned):
			self.i=1
			self.reassigned_points=self.reassign_point_values('elongated',self.f_elongated)
			self.reassigned=True
			self.show_map(self.reassigned_points,True)

		else:
			self.reassigned=False
			self.show_map(self.points,False)


	def read_points(self,filename):
		f=open(filename,'r')
		self.points=dict()

		for line in f:
			if '---' not in line:
				if 'Original' in line:
					temp_tags=dict()
					label=line.split(' ')[1]
					x=line.split(' ')[2]
					y=line.split(' ')[3]
					z=line.split(' ')[4]

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
					self.points[label]=temp_tags
		print(self.points)
		f.close()


	def read_percs(self,filename):
		f=open(filename,'r')
		percs=dict()

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

		f.close()
		return percs

	def setup_grid(self):

		self.x, self.y = np.mgrid[-0.8:0.8:.05, -0.8:0.8:.05]
		self.pos = np.empty(self.x.shape + (2,))
		self.pos[:, :, 0] = self.x; self.pos[:, :, 1] = self.y

		rv1 = multivariate_normal([-0.7, -0.7],[[0.25,0],[0,0.25]])
		rv2 = multivariate_normal([0.7,0.7],[[0.25,0],[0,0.25]])

		Z1=rv1.pdf(self.pos)
		Z2=rv2.pdf(self.pos)
		self.Ze=Z1+Z2
		self.f_elongated = interpolate.interp2d(self.x, self.y, self.Ze, kind='cubic')

	def setup_fig(self):

		self.fig = plt.figure()
		self.ax = self.fig.gca(projection='3d')
		self.fig.canvas.mpl_connect('button_press_event', self.press)
		self.ax.set_xlabel('x')
		self.ax.set_ylabel('y')
		self.ax.set_zlabel('z')

	def press(self,event):
		sys.stdout.flush()
		print('A')
		print(self.i)
		print(self.reassigned)
		print(self.actions[self.i-1])
		print(self.positions)

		if(self.i<len(self.positions)-1):


   			if self.reassigned:
   				print('Here')
   				x2=float(self.reassigned_points[self.positions[self.i]]['x'])
   				x1=float(self.reassigned_points[self.positions[self.i-1]]['x'])
   				y2=float(self.reassigned_points[self.positions[self.i]]['y'])
   				y1=float(self.reassigned_points[self.positions[self.i-1]]['y'])
   				z2=float(self.reassigned_points[self.positions[self.i]]['generated'])
   				z1=float(self.reassigned_points[self.positions[self.i-1]]['generated'])
   			else:
   				x2=float(self.points[self.positions[self.i]]['x'])
   				x1=float(self.points[self.positions[self.i-1]]['x'])
   				y2=float(self.points[self.positions[self.i]]['y'])
   				y1=float(self.points[self.positions[self.i-1]]['y'])

   				z2=float(self.points[self.positions[self.i]]['z'])
   				z1=float(self.points[self.positions[self.i-1]]['z'])

   			if(self.actions[self.i-1]=='south'):
      				color='g'
      			elif self.actions[self.i-1]=='east':
      				color='r'
      			elif self.actions[self.i-1]=='north':
      				color='y'
      			else:
      				color='b'


   			


      		q=np.sqrt((x2-x1)**2+(y2-y1)**2)
      		ux=(x2-x1)/q
      		uy=(y2-y1)/q
      		uz=(z2-z1)/q

      		for l in np.arange(0,q,.01):
      			ppx=x1+l*ux
      			ppy=y1+l*uy
      			ppz=z1+l*uz

      			self.ax.scatter(ppx,ppy,ppz,s=1,color=color)
      		self.i+=1
      		self.fig.canvas.draw()




	def setup_entries(self,positions,actions):
		self.positions=positions
		self.actions=actions



	def read_percentages(self):
		elong_percs=self.read_percs(self.elongated_filename)
		livarno_percs=self.read_percs(self.livarno_filename)
		mush_percs=self.read_percs(self.mushroom_filename)
		std_percs=self.read_percs(self.standard_filename)

		self.percs.append(elong_percs)
		self.percs.append(livarno_percs)
		self.percs.append(mush_percs)
		self.percs.append(std_percs)
    

   	def show_map(self,points,reassigned):

   		px=[]
   		py=[]
   		pz=[]
   		label=[]

   		surf1=self.ax.plot_surface(self.x,self.y,self.Ze, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.coolwarm,alpha=0.7)

   		for k,v in points.items():
   			px.append(float(v['x']))
   			py.append(float(v['y']))

   			label.append(k)

   			if reassigned:
   				pz.append(float(v['generated']))
   				self.ax.text(float(v['x']),float(v['y']),float(v['generated']),str(k),size='large')
   			else:
   				pz.append(float(v['z']))
   				self.ax.text(float(v['x']),float(v['y']),float(v['z']),str(k),size='large')

   		self.ax.grid('on')
   		self.ax.scatter(px,py,pz)

   		'''for i in range(1,len(self.positions)):
   			x2=float(points[self.positions[i]]['x'])
   			x1=float(points[self.positions[i-1]]['x'])

   			y2=float(points[self.positions[i]]['y'])
   			y1=float(points[self.positions[i-1]]['y'])


   			if reassigned:
   				z2=float(points[self.positions[i]]['generated'])
   				z1=float(points[self.positions[i-1]]['generated'])
   			else:
   				z2=float(points[self.positions[i]]['z'])
   				z1=float(points[self.positions[i-1]]['z'])


   			q=np.sqrt((x2-x1)**2+(y2-y1)**2)
   			ux=(x2-x1)/q
   			uy=(y2-y1)/q
   			uz=(z2-z1)/q

   			for l in np.arange(0,q,.01):
   				ppx=x1+l*ux
   				ppy=y1+l*uy
   				ppz=z1+l*uz

   				if(self.actions[i-1]=='south'):
   					color='g'
   				elif self.actions[i-1]=='east':
   					color='r'
   				elif self.actions[i-1]=='north':
   					color='y'
   				else:
   					color='b'

   				self.ax.scatter(ppx,ppy,ppz,s=1,color=color)'''

   		plt.show()

   	def reassign_point_values(self,bulb_type,func):
   		reassigned_points=dict()

   		for k,v in self.points.items():
   			px=float(v['x'])
   			py=float(v['y'])

   			temp_values=dict()
   			temp_values['x']=px
   			temp_values['y']=py
   			temp_values['generated']=func(px,py)[0]
   			reassigned_points[k]=temp_values

   		return reassigned_points











def show_map(fig,points,positions,actions,reassigned):

	px=[]
	py=[]
	pz=[]
	label=[]

	fig1.canvas.mpl_connect('key_press_event', press)

	for k,v in points.items():

		px.append(float(v['x']))
		py.append(float(v['y']))

		label.append(k)

		if reassigned:
			pz.append(float(v['generated']))
			fig.text(float(v['x']),float(v['y']),float(v['generated']),str(k),size='large')

		else:
			pz.append(float(v['z']))
			fig.text(float(v['x']),float(v['y']),float(v['z']),str(k),size='large')




	print(px)
	print(py)
	print(pz)
	print(label)

	fig.grid('on')
	fig.scatter(px,py,pz)

	fig.set_xlabel('x')
	fig.set_ylabel('y')
	fig.set_zlabel('z')

	for i in range(1,len(positions)):

		x2=float(points[positions[i]]['x'])
		x1=float(points[positions[i-1]]['x'])

		y2=float(points[positions[i]]['y'])
		y1=float(points[positions[i-1]]['y'])


		if reassigned:
			z2=float(points[positions[i]]['generated'])
			z1=float(points[positions[i-1]]['generated'])
		else:
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


			fig.scatter(ppx,ppy,ppz,s=1,color=color)

	plt.show()


### Fetch and create surface ###

"""x, y = np.mgrid[-0.8:0.8:.05, -0.8:0.8:.05]
pos = np.empty(x.shape + (2,))
pos[:, :, 0] = x; pos[:, :, 1] = y

###Elongated###
rv1 = multivariate_normal([-0.7, -0.7],[[0.25,0],[0,0.25]])
rv2 = multivariate_normal([0.7,0.7],[[0.25,0],[0,0.25]])

Z1=rv1.pdf(pos)
Z2=rv2.pdf(pos)
Ze=Z1+Z2
f_elongated = interpolate.interp2d(x, y, Ze, kind='cubic')


fig1 = plt.figure()

ax1 = fig1.gca(projection='3d')

surf1=ax1.plot_surface(x, y, Ze, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.coolwarm,alpha=0.7)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')

def read_points(self,filename):
		f=open(filename,'r')
		self.points=dict()

		for line in f:
			if '---' not in line:
				if 'Original' in line:
					temp_tags=dict()
					label=line.split(' ')[1]
					x=line.split(' ')[2]
					y=line.split(' ')[3]
					z=line.split(' ')[4]

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
				self.points[label]=temp_tags

		print(self.points)
        f.close()




def read_percs(self,filename):
		f=open(filename,'r')
    	percs=dict()

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
    	print('Aa')
    	f.close()









"""


################################


### Fetch needed data###

points_filename='/home/cptd/c/graph.txt'
elongated_filename='/home/cptd/test_results/gen_elongated.txt'
livarno_filename='/home/cptd/test_results/gen_livarno.txt'
mushroom_filename='/home/cptd/test_results/gen_mushroom.txt'
standard_filename='/home/cptd/test_results/gen_standard.txt'
bulbs=['elongated','livarno','mushroom','standard']

#points=read_points(points_filename)
#points=temp
positions=['11','10','21','22','22','10','9','20','21','22']
actions=['south','south','south','south','north','north','west','south','east']

plotter=MovementPlotter(points_filename,elongated_filename,livarno_filename,mushroom_filename,standard_filename)
plotter.setup_grid()
plotter.setup_fig()
plotter.setup_entries(positions,actions)
plotter.show(True)


#reassigned_points=reassign_point_values(points,percs[0],'elongated',f_elongated)
#print(reassigned_points)


#show_map(ax1,reassigned_points,positions,actions,True)

########################


"""
Show how to connect to keypress events
"""

