import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import sys
from matplotlib import cm
from scipy.stats import multivariate_normal
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.interpolate import griddata
from scipy.interpolate import *

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

	def show(self,reassigned,start,stop,final):

		self.reassigned=reassigned
		self.i=0

		if(self.reassigned):
			self.reassigned_points=self.reassign_point_values('elongated',self.f_elongated)
			self.show_map(self.reassigned_points,start,stop,final)

		else:
			self.show_map(self.points,start,stop,final)


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
					neighbours=[]
					temp=line.split(' ')

					for i in range(1,len(temp)):
						neighbours.append(temp[i].strip())
					temp_tags['neighbours']=neighbours
				else:
					self.points[label]=temp_tags
		f.close()

	def get_points(self):
		return self.points


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

		#Elongated#
		#rv1 = multivariate_normal([-0.7, -0.7],[[0.25,0],[0,0.25]])
		#rv2 = multivariate_normal([0.7,0.7],[[0.25,0],[0,0.25]])

		#rv1 = multivariate_normal([-0.7, -0.7],[[0.25,0],[0,0.25]])
		#rv2 = multivariate_normal([0.7, 0.7],[[0.25,0],[0,0.25]])
		#rv3 = multivariate_normal([0.2, 0],[[0.01,0],[0,0.2]])

		self.Ze=(1-self.x)**2.*np.exp(-(self.x**2) - (self.y+1)**2)- 10*(self.x/5 - self.x**3 - self.y**5)*np.exp(-self.x**2-self.y**2)- 1/3*np.exp(-(self.x+1)**2 - self.y**2)  

		#Livarno#
		#rv1 = multivariate_normal([0.6, 0.0],[[0.25,0],[0,0.25]])
		#rv2 = multivariate_normal([-0.8,0],[[0.25,0],[0,0.25]])


		#Z1=rv1.pdf(self.pos)
		#Z2=rv2.pdf(self.pos)
		#Z3=rv3.pdf(self.pos)

		#self.Ze=Z1+Z2*Z3-0.2
		self.f_elongated = interpolate.interp2d(self.x, self.y, self.Ze, kind='cubic')

	def setup_fig(self,step=False):

		self.step=step
		self.fig = plt.figure()
		self.ax = self.fig.gca(projection='3d')

		if self.step:
			self.fig.canvas.mpl_connect('button_press_event', self.press)

		self.ax.set_xlabel('x')
		self.ax.set_ylabel('y')
		self.ax.set_zlabel('z')





	def setup_entries(self,positions,actions):
		self.positions=positions
		self.actions=actions

	def press(self,event):

		print('We are here to make some noise!')
		sys.stdout.flush()
		self.i+=1
		print('Current :'+str(self.i)+' Total Lenght :'+str(len(self.positions)))

		if(self.i<len(self.positions)):


   			if self.reassigned:
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
      			elif self.actions[self.i-1]=='west':
      				color='b'


      		q=np.sqrt((x2-x1)**2+(y2-y1)**2)

      		if q>0:
      			ux=(x2-x1)/q
      			uy=(y2-y1)/q
      			uz=(z2-z1)/q

      			for l in np.arange(0,q,.01):
      				ppx=x1+l*ux
      				ppy=y1+l*uy
      				ppz=z1+l*uz
      				self.ax.scatter(ppx,ppy,ppz,s=1,color=color)
      		else:
      			print('Stay in the same positions')

      		self.fig.canvas.draw()






	def read_percentages(self):
		elong_percs=self.read_percs(self.elongated_filename)
		livarno_percs=self.read_percs(self.livarno_filename)
		mush_percs=self.read_percs(self.mushroom_filename)
		std_percs=self.read_percs(self.standard_filename)

		self.percs.append(elong_percs)
		self.percs.append(livarno_percs)
		self.percs.append(mush_percs)
		self.percs.append(std_percs)
    

   	def show_map(self,points,start,stop,final=False):

   		px=[]
   		py=[]
   		pz=[]
   		label=[]

   		surf1=self.ax.plot_surface(self.x,self.y,self.Ze, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.coolwarm,alpha=0.7)

   		for k,v in points.items():
   			px.append(float(v['x']))
   			py.append(float(v['y']))
   			pz.append(float(v['z']))

   			label.append(k)
   			self.ax.text(float(v['x']),float(v['y']),float(v['z']),str(k),size='large')

   		self.ax.grid('on')
   		self.ax.scatter(px,py,pz)

   		if final:
   			start_point=points[start]
   			end_point=points[stop]

   			self.ax.text(float(start_point['x']),float(start_point['y']),float(start_point['z']),str(start)+' start',size='large')
   			self.ax.text(float(end_point['x']),float(end_point['y']),float(end_point['z']),str(stop)+' end',size='large')





   		if not self.step:
   			for i in range(1,len(self.positions)):
   				x2=float(points[self.positions[i]]['x'])
   				x1=float(points[self.positions[i-1]]['x'])
   				y2=float(points[self.positions[i]]['y'])
   				y1=float(points[self.positions[i-1]]['y'])
   				z2=float(points[self.positions[i]]['z'])
   				z1=float(points[self.positions[i-1]]['z'])

   				if self.actions[i-1]=='decision':
   						self.ax.text(x2,y2,z2,' * * *',size='large')

	  			
   				q=np.sqrt((x2-x1)**2+(y2-y1)**2)

   				if q>0:
   					ux=(x2-x1)/q
   					uy=(y2-y1)/q
   					uz=(z2-z1)/q

   					if(self.actions[i-1]=='south'):
   						color='g'
   					elif self.actions[i-1]=='east':
   						color='r'
   					elif self.actions[i-1]=='north':
   						color='y'
   					elif self.actions[i-1]=='west':
   						color='b'


   					for l in np.arange(0,q,.01):
   						ppx=x1+l*ux
   						ppy=y1+l*uy
   						ppz=z1+l*uz

   						self.ax.scatter(ppx,ppy,ppz,s=1,color=color)

   		plt.show()


   	def reassign_point_values(self,bulb_type,func):
   		reassigned_points=dict()

   		for k,v in self.points.items():
   			px=float(v['x'])
   			py=float(v['y'])

   			temp_values=dict()
   			temp_values['x']=px
   			temp_values['y']=py
   			temp_values['z']=func(px,py)[0]
   			temp_values['generated']=func(px,py)[0]
   			reassigned_points[k]=temp_values

   		return reassigned_points



	''' 		
   				for l in np.arange(0,q,.01):
   					ppx=x1+l*ux
   					ppy=y1+l*uy
   					ppz=z1+l*uz

   				self.ax.scatter(ppx,ppy,ppz,s=1,color=color)'''