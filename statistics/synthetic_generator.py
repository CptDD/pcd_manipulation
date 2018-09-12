import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import multivariate_normal
import numpy as np
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from scipy.interpolate import griddata
from scipy.interpolate import *


def write_points(filename,points,bulb_type,elo,liv,mush,std):
    out=open(filename,'w')

    for k,v in points.items():

        px=float(points[str(k)]['x'])
        py=float(points[str(k)]['y'])
        pz=float(points[str(k)]['z'])

        percentage_e=elo(px,py)
        percentage_l=liv(px,py)
        percentage_m=mush(px,py)
        percentage_s=std(px,py)

        out.write('Bulb:'+'\t\t'+bulb_type+'\n')
        out.write('Label:'+'\t\t'+str(k)+'\n')
        out.write('Percentage_e:'+'\t'+str(percentage_e[0])+'\n')
        out.write('Percentage_l:'+'\t'+str(percentage_l[0])+'\n')
        out.write('Percentage_m:'+'\t'+str(percentage_m[0])+'\n')
        out.write('Percentage_s:'+'\t'+str(percentage_s[0])+'\n')
        out.write('---'+'\n')

    out.close()


def reassign_point_values(points,percs,bulb_type,func):

	print('Reasigning for :'+str(bulb_type))

	reassigned_points=dict()

	for k,v in points.items():

		px=float(v['x'])
		py=float(v['y'])

		
		print('For :'+str(k)+'\tmeasured is :'+str(percs[k][bulb_type])+'\tinterpolated value :'+str(func(px,py)[0]))

		temp_values=dict()
		temp_values['x']=px
		temp_values['y']=py
		temp_values['generated']=func(px,py)[0]
		temp_values['measured']=percs[k][bulb_type]
		reassigned_points[k]=temp_values


	return reassigned_points



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
                neighbours=[]
                temp=line.split(' ')
                for i in range(1,len(temp)):
                    neighbours.append(temp[i].strip())

                temp_tags['neighbours']=neighbours

            else:
                points[label]=temp_tags
        

    f.close()
    return points

def plot_points(fig,points):

	for k,v in points.items():

		px=float(v['x'])
		py=float(v['y'])
		pz=float(v['generated'])

		fig.scatter(px,py,pz,s=1,color='black')
		fig.text(px,py,pz,str(k),size='large')





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

    f.close()
    return percs


points=read_points('/home/cptd/c/graph.txt')
percs=[]
percs_elongated=read_percentages('/home/cptd/test_results/noise_elongated.txt')
percs_livarno=read_percentages('/home/cptd/test_results/noise_livarno.txt')
percs_mushroom=read_percentages('/home/cptd/test_results/noise_mushroom.txt')
percs_standard=read_percentages('/home/cptd/test_results/noise_standard.txt')

percs.append(percs_elongated)
percs.append(percs_livarno)
percs.append(percs_mushroom)
percs.append(percs_standard)

x, y = np.mgrid[0.8:-0.8:-.05, 0.8:-0.8:-.05]
pos = np.empty(x.shape + (2,))
pos[:, :, 0] = x; pos[:, :, 1] = y


###Elongated###
rv1 = multivariate_normal([-3, -3],[[0.25,0],[0,0.25]])
rv2 = multivariate_normal([3,3],[[0.25,0],[0,0.25]])

#rv1 = multivariate_normal([-0.7, -0.7],[[0.25,0],[0,0.25]])
#rv2 = multivariate_normal([0.7,0.7],[[0.25,0],[0,0.25]])

Z1=rv1.pdf(pos)
Z2=rv2.pdf(pos)
Ze=Z1+Z2

#Ze=(1-x)**2.*np.exp(-(x**2) - (y+1)**2)- 10*(x/5 - x**3 - y**5)*np.exp(-x**2-y**2)- 1/3*np.exp(-(x+1)**2 - y**2)  


f_elongated = interpolate.interp2d(x, y, Ze, kind='cubic')


##############

###Livarno###
rv3 = multivariate_normal([-0.9, -0.9],[[0.1,0],[0.0,0.2]])
rv4 = multivariate_normal([0.9,0.9],[[0.2,0.0],[0,0.1]])


#rv3 = multivariate_normal([0.6, 0.0],[[0.25,0],[0,0.25]])
#rv4 = multivariate_normal([-0.8,0],[[0.25,0],[0,0.25]])


Z3=rv3.pdf(pos)
Z4=rv4.pdf(pos)
Zl=Z3+Z4


f_livarno = interpolate.interp2d(x, y, Zl, kind='cubic')
##############

###Mushroom###
rv5 = multivariate_normal([-1, -1],[[0.3,0],[0.0,0.05]])
rv6 = multivariate_normal([1,1],[[0.8,0],[0,0.1]])

#rv5 = multivariate_normal([-0.2, -0.2],[[0.2,0],[0.0,0.05]])
#rv6 = multivariate_normal([0.5,0.5],[[0.1,0],[0,0.1]])


#Z5=rv5.pdf(pos)
#Z6=rv6.pdf(pos)
#Zm=Z5+Z6+0.1


#Zm= x**2 - y/2+0.2
Zm=x * np.exp(-x**2 - y**2)

f_mushroom = interpolate.interp2d(x, y, Zm, kind='cubic')
##############


###Standard###
rv7 = multivariate_normal([-2, -2],[[0.01,0],[0.0,0.1]])
rv8 = multivariate_normal([2,2],[[0.01,0],[0,0.1]])

#rv7 = multivariate_normal([0.3, -0.8],[[0.01,0],[0.0,0.1]])
#rv8 = multivariate_normal([0.3,0.8],[[0.01,0],[0,0.1]])


Z7=rv7.pdf(pos)
Z8=rv8.pdf(pos)
Zs=Z7+Z8


Zs=np.sin(np.sqrt(2*x**2  + 2*y**2))-0.1


f_standard = interpolate.interp2d(x, y, Zs, kind='cubic')
##############


fig1 = plt.figure()
ax1 = fig1.gca(projection='3d')


surf1=ax1.plot_surface(x, y, Ze, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.coolwarm,alpha=0.7)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')

fig1.colorbar(surf1, shrink=0.5, aspect=5)

plt.title('Elongated generated function')



fig2=plt.figure()
ax2 = fig2.gca(projection='3d')
surf2=ax2.plot_surface(x, y, Zl, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.coolwarm,alpha=0.7)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')
ax2.grid(True)

fig2.colorbar(surf2, shrink=0.5, aspect=5)

plt.title('Livarno generated function')



fig3 = plt.figure()
ax3 = fig3.gca(projection='3d')
surf3=ax3.plot_surface(x, y, Zm, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.coolwarm,alpha=0.7)
ax3.set_xlabel('x')
ax3.set_ylabel('y')
ax3.set_zlabel('z')
fig3.colorbar(surf3, shrink=0.5, aspect=5)
plt.title('Mushroom generated function')



fig4 = plt.figure()
ax4  = fig4.gca(projection='3d')
surf4=ax4.plot_surface(x, y, Zs, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.coolwarm,alpha=0.7)
ax4.set_xlabel('x')
ax4.set_ylabel('y')
ax4.set_zlabel('z')
fig4.colorbar(surf4, shrink=0.5, aspect=5)
plt.title('Standard generated function')

#plt.show()


reassigned_points_e=reassign_point_values(points,percs[0],'elongated',f_elongated)
reassigned_points_l=reassign_point_values(points,percs[1],'livarno',f_livarno)
reassigned_points_m=reassign_point_values(points,percs[2],'mushroom',f_mushroom)
reassigned_points_s=reassign_point_values(points,percs[3],'standard',f_standard)


plot_points(ax1,reassigned_points_e)
plot_points(ax2,reassigned_points_l)
plot_points(ax3,reassigned_points_m)
plot_points(ax4,reassigned_points_s)


plt.show()


elongated_filename='/home/cptd/test_results/gen_elongated.txt'
livarno_filename='/home/cptd/test_results/gen_livarno.txt'
mushroom_filename='/home/cptd/test_results/gen_mushroom.txt'
standard_filename='/home/cptd/test_results/gen_standard.txt'


#write_points(elongated_filename,points,'elongated',f_elongated,f_livarno,f_mushroom,f_standard)
#write_points(livarno_filename,points,'livarno',f_elongated,f_livarno,f_mushroom,f_standard)
#write_points(mushroom_filename,points,'mushroom',f_elongated,f_livarno,f_mushroom,f_standard)
#write_points(standard_filename,points,'standard',f_elongated,f_livarno,f_mushroom,f_standard)