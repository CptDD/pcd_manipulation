#ifndef DISTRIB_EM
#define DISTRIB_EM

#include <iostream>
#include <pcd_processor/N2.hpp>


class DistributionEntryMultiple
{

private:
	string label;
	int type;
	int total_apperances;
	vector<int> positives;
	vector<double> percentages;
public:
	DistributionEntryMultiple(string label,int type,int total_apperances,int number_of_classes)
	{
		this->label=label;
		this->type=type;
		this->total_apperances=total_apperances;
		this->positives.resize(number_of_classes);
		this->percentages.resize(number_of_classes);
	}

	DistributionEntryMultiple()
	{
		label="default";
		type=0;
		total_apperances=0;

	}

	void set_label(string label)
  	{
    	this->label=label;
  	}

  	void set_type(int type)
  	{
    	this->type=type;
  	}

  	void set_total(int total)
	{	
    	this->total_apperances=total;
  	}

  	int get_total()
  	{
	    return total_apperances;
  	}

  	int get_type()
  	{
	    return type;
  	}

  	void plus_total()
  	{
    	total_apperances++;
  	}

  	void set_number_of_classes(int number_of_classes)
  	{
  		positives.resize(number_of_classes);
  		percentages.resize(number_of_classes);
  	}

  	void plus_positives(int pos)
  	{
  		this->positives[pos]++;
  	}

 	
 	vector<double> get_percentages()
 	{
 		return percentages;
 	}

 	vector<int> get_positives()
 	{
 		return positives;
 	}

 	void show_percentages()
 	{
 		cout<<"Percentages :";
 		for(int i=0;i<percentages.size();i++)
 		{
 			cout<<percentages[i]<<" ";
 		}
 		cout<<endl;
 	}

 	void update_percentage()
 	{
 		for(int i=0;i<percentages.size();i++)
 		{
 			percentages[i]=(double)(positives[i])/total_apperances;
 		}
 	}


 	void show_positives()
 	{
 		cout<<"Positives :";
 		for(int i=0;i<positives.size();i++)
 		{
 			cout<<positives[i]<<" ";
 		}
 		cout<<endl;
 	}
  
  	string get_label()
  	{
    	return label;
  	}

  	void compute_percentage()
  	{
    	//double perc=(double)positives/total_apperances;
    	//perc*=100;
    	//cout<<"Percentage :"<<perc<<endl;
      	//this->set_percentage(perc);
  	}

  	void show()
  	{
    	 cout<<"Label:"<<label<<"\t"<<"Type:"<<type<<"\t"<<"Good:"<<positives.size()<<"\t"<<"How many:"
     	<<total_apperances<<"\t"<<"Percentage:"<<percentages.size()<<endl;
     	this->show_positives();
     	this->show_percentages();
  	}


};

#endif;