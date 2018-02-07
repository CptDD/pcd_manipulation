#!/usr/bin/python


class BayesianUpdater:
	def __init__(self):
		self.distro=dict()
		self.belief=[0.25,0.25,0.25,0.25]

	def fetch(self,file):
		f=open(file,"r")

		values=[]
		i=0

		for line in f:
			if "===" not in line:
				if i%6==0:
					viewpoint=line.split('/')[7]
				else:
					temp_tags=dict()
					tags=line.split('/t')
					temp_tags['label']=tags[0]
					temp_tags['type']=tags[1]
					temp_tags['positives']=tags[2]
					temp_tags['total']=tags[3]
					temp_tags['percentage']=tags[4].split('\n')[0]
					values.append(temp_tags)
			else:
				self.distro[viewpoint]=values
				values=list()
			i+=1

	def get_distro(self):
		return self.distro

	def update(self):
		for i in self.distribution:
			if int(i['type']==observed_type):
				print(i['label'])
				val=float(i['percentage'])*belief[observed_type]
				sum_b=sum(belief)

				for j in range(0,4):
					belief[j]/=sum_b

	

		
