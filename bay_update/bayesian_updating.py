#!/usr/bin/python


def process_line(line):
	print("Original line :"+line)

def fetch():
	f=open("out.txt","r")

	distro=dict()

	values=[]
	i=0
	for line in f:
		if "===" not in line:
			if i%6==0:
				viewpoint=line.split('/')[7]
			else:
				temp_tags=dict()
				tags=line.split('\t')
				temp_tags['label']=tags[0]
				temp_tags['type']=tags[1]
				temp_tags['positives']=tags[2]
				temp_tags['total']=tags[3]
				temp_tags['percentage']=tags[4].split('\n')[0]
				values.append(temp_tags)
		else:
			distro[viewpoint]=values
			values=list()
		i+=1

	return distro


		
	
def main():
	update()


def update():
	distribution=fetch()
	belief=[1,1,1,1]

	observed_type=1
	observed_viewpoint="side_right"

	


if __name__=='__main__':
	main()