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

"""Function used to write the generated observation probabilites to a file.
elo,liv,mush and std are the interpolation function used to compute the respective 
values for each point.
"""

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



def read_points(filename):
    """Function that reads points from a file."""
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


points=read_points('/home/cptd/c/graph.txt')




x, y = np.mgrid[-0.7:0.7:.05, -0.7:0.7:.05]
pos = np.empty(x.shape + (2,))
pos[:, :, 0] = x; pos[:, :, 1] = y


###Elongated generated function and computation of interpolation function###
rv1 = multivariate_normal([-0.7, -0.7],[[0.25,0],[0,0.25]])
rv2 = multivariate_normal([0.7,0.7],[[0.25,0],[0,0.25]])

Z1=rv1.pdf(pos)
Z2=rv2.pdf(pos)
Ze=Z1+Z2

f_elongated = interpolate.interp2d(x, y, Ze, kind='cubic')


##############

###Livarno generated function and computation of interpolation function###
rv3 = multivariate_normal([-0.7, -0.7],[[0.1,0],[0.0,0.2]])
rv4 = multivariate_normal([0.7,0.7],[[0.1,0],[0,0.1]])

Z3=rv3.pdf(pos)
Z4=rv4.pdf(pos)
Zl=Z3+Z4

f_livarno = interpolate.interp2d(x, y, Zl, kind='cubic')
##############

###Mushroom generated function and computation of interpolation function###
rv5 = multivariate_normal([-1, -1],[[0.2,0],[0.0,0.05]])
rv6 = multivariate_normal([1,1],[[0.1,0],[0,0.1]])

Z5=rv5.pdf(pos)
Z6=rv6.pdf(pos)
Zm=Z5+Z6

f_mushroom = interpolate.interp2d(x, y, Zm, kind='cubic')
##############


###Standard generated function and computation of interpolation function###
rv7 = multivariate_normal([-2, -2],[[0.01,0],[0.0,0.1]])
rv8 = multivariate_normal([2,2],[[0.01,0],[0,0.1]])

Z7=rv7.pdf(pos)
Z8=rv8.pdf(pos)
Zs=Z7+Z8+0.1

f_standard = interpolate.interp2d(x, y, Zs, kind='cubic')
##############


"""Plotting the generated functions"""
fig1 = plt.figure()
ax1 = fig1.gca(projection='3d')


ax1.plot_surface(x, y, Ze, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.viridis)
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
plt.title('Elongated generated function')



fig2=plt.figure()
ax2 = fig2.gca(projection='3d')
ax2.plot_surface(x, y, Zl, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.viridis)
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')
ax2.grid(True)
plt.title('Livarno generated function')



fig3 = plt.figure()
ax3 = fig3.gca(projection='3d')
ax3.plot_surface(x, y, Zm, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.viridis)
ax3.set_xlabel('x')
ax3.set_ylabel('y')
ax3.set_zlabel('z')
plt.title('Mushroom generated function')



fig4 = plt.figure()
ax4  = fig4.gca(projection='3d')
ax4.plot_surface(x, y, Zs, rstride=3, cstride=3, linewidth=1, antialiased=True,
               cmap=cm.viridis)
ax4.set_xlabel('x')
ax4.set_ylabel('y')
ax4.set_zlabel('z')
plt.title('Standard generated function')


plt.show()

"""Writing the values"""
elongated_filename='/home/cptd/test_results/gen_elongated.txt'
write_points(elongated_filename,points,'elongated',f_elongated,f_livarno,f_mushroom,f_standard)