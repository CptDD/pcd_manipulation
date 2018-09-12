#!/usr/bin/env python

import matplotlib.pyplot as plt

plt.style.use('ggplot')

'''elongated_aems2=[0.25,0.327501,0.409079,0.918669,0.11605,0.994707,0.994707, 0.999613,0.999872,0.999992,0.999999]
elongated_despot=[0.25,0.327501,0.627319,0.716745,0.843368,0.904038,0.992543,0.995154,0.991695,0.995743,0.998797]'''

elongated_bstate=[0.25,0.661417,0.969275,0.996438,0.99,0.99,0.999741,0.999987,0.999998,1,1]
livarno_bstate=[0.25,0.0787402,0.0144978,0.0017743,0.000222468,0.000666548,8.34459e-05,1.34986e-05,1.68734e-06,2.72152e-07,7.77578e-08]
mushroom_bstate=[0.25,0.188976,0,0,0,0,0,0,0,0]
standard_bstate=[0.25, 0.0708661,0.0708661,0.0162267,0.000280123,0.00111905,0.000175119,0,0,0,0]
elongated_ig=[]

livarno_ig=[]
mushroom_ig=[]
standard_ig=[]
expected_ig=[]
obs_vec=['oelongated','oelongated','oelongated','oelongated','omushroom','olivarno','oelongated','oelongated','oelongated','oelongated']
action_vec=['south','north','south','north','north','west','north','west','north','east']
pos_vec=['vp11','vp24','vp11','vp24','vp11','vp13','vp11','vp13','vp11','vp13','vp12']


fig=plt.figure()
ax1 = fig.add_subplot(1,1,1)
'''ax1.plot(elongated_aems2, marker=r'o', color=u'blue', linestyle='--',\
label='Aems2 belief state')
ax1.plot(elongated_despot, marker=r'o', color=u'red', linestyle='--',\
label='Despot belief state')'''
ax1.plot(elongated_bstate, marker=r'o', color=u'red', linestyle='--',\
label='Elongated belief state')
ax1.plot(livarno_bstate, marker=r'o', color=u'blue', linestyle='--',\
label='Livarno belief state')
ax1.plot(mushroom_bstate, marker=r'o', color=u'green', linestyle='--',\
label='Mushroom belief state')
ax1.plot(standard_bstate, marker=r'o', color=u'orange', linestyle='--',\
label='Standard belief state')
ax1.xaxis.set_ticks_position('bottom')
ax1.yaxis.set_ticks_position('left')
ax1.set_title('Belief state evolution standard')
plt.legend(loc='best')


fig2=plt.figure()
ax2=fig2.add_subplot(1,1,1)
ax2.plot(elongated_ig, marker=r'o', color=u'blue', linestyle='--',\
label='Elongated information gain')
ax2.plot(livarno_ig, marker=r'o', color=u'red', linestyle='--',\
label='Livarno information gain')
ax2.plot(mushroom_ig, marker=r'o', color=u'green', linestyle='--',\
label='Mushroom information gain')
ax2.plot(standard_ig, marker=r'o', color=u'orange', linestyle='--',\
label='Standard information gain')
ax2.xaxis.set_ticks_position('bottom')
ax2.yaxis.set_ticks_position('left')
ax2.set_title('Information gain')
plt.legend(loc='best')

fig3=plt.figure()
ax3=fig3.add_subplot(1,1,1)
ax3.bar(range(len(expected_ig)),expected_ig, align='center', color='darkblue')
#ax3.plot(expected_ig,marker=r'o',color=u'blue')
ax3.xaxis.set_ticks_position('bottom')
ax3.yaxis.set_ticks_position('left')
plt.xlabel('Step')
plt.ylabel('Expected information gain')
plt.title('Expected information gain per step')

fig4=plt.figure()
ax4=fig4.add_subplot(1,1,1)
ax4.plot(obs_vec,marker=r'o',color=u'blue')
label='Observation vector'
ax4.plot(pos_vec,marker=r'o',color=u'red')
label='Position vector'
ax4.plot(action_vec,marker=r'o',color=u'yellow')
label='actions'
ax4.xaxis.set_ticks_position('bottom')
ax4.yaxis.set_ticks_position('left')
plt.xlabel('Step')
plt.ylabel('Sequence of observations')
plt.title('Sequence of observations')

plt.show()

