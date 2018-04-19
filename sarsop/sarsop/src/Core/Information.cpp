#include <iostream>
#include <cmath>
#include <cstring>
#include "Information.h"


using namespace std;


void Information::go()
{
    cout<<"Go go go!"<<endl;

}


void Information::set_pre(SharedPointer<SparseVector> &sp)
{
        cout<<"Initializing pre!"<<endl;
		for(int i=0;i<sp->data.size();i++)
		{
			SparseVector_Entry ee=sp->data[i];
            
            switch(ee.index)
            {
                case 0:
                {
                    pre_map["elongated"]=ee.value;
                    break;
                }

                case 1:
                {
                   
                    pre_map["livarno"]=ee.value;
                    break;
                }

                case 2:
                {
                    pre_map["mushroom"]=ee.value;
                    break;
                }
                case 3:
                {
                    pre_map["standard"]=ee.value;
                    break;
                }
            }
		}

        this->check_map(pre_map);
}

void Information::set_post(SharedPointer<SparseVector> &sp)
{
    cout<<"Initializing post!"<<endl;
		for(int i=0;i<sp->data.size();i++)
		{
			SparseVector_Entry ee=sp->data[i];
            
            switch(ee.index)
            {
                case 0:
                {
                
                    post_map["elongated"]=ee.value;
                    break;
                }

                case 1:
                {
                   
                    post_map["livarno"]=ee.value;
                    break;
                }

                case 2:
                {
                    post_map["mushroom"]=ee.value;
                    break;
                }
                case 3:
                {
                    post_map["standard"]=ee.value;
                    break;
                }
            }
		}

        this->check_map(post_map);
}


void Information::check_map(map<string,double> &values)
{
    map<string,double>::iterator it;

    it=values.find("elongated");

    if(it==values.end())
    {
        values["elongated"]=0;
    }

    it=values.find("livarno");

    if(it==values.end())
    {
        values["livarno"]=0;
    }

    it=values.find("mushroom");

    if(it==values.end())
    {
        values["mushroom"]=0;
    }

    it=values.find("standard");

    if(it==values.end())
    {
        values["standard"]=0;
    }
}


map<string,double> Information::compute_information_gain()
{
	map<string,double> information_gain;
    map<string,double>::iterator it;

    for(it=pre_map.begin();it!=pre_map.end();++it)
    {
        double pre_val=it->second;
        double post_val=post_map.at(it->first);

        if(pre_val!=0 && post_val!=0)
        {
            double temp_div=post_val/pre_val;
            double temp_val=post_val*log2(temp_div);

            information_gain[it->first]=temp_val;
        }

        if(pre_val==0 || post_val==0)
        {
            information_gain[it->first]=0;
        }

    }

    cout<<"===Information gain==="<<endl;
    for(it=information_gain.begin();it!=information_gain.end();++it)
    {
        cout<<it->first<<" "<<it->second<<endl;
    }
    cout<<"======================"<<endl;

    return information_gain;
}

double Information::compute_expected_information_gain()
{
    map<string,double> information_gain=this->compute_information_gain();
    map<string,double>::iterator it;

    double eig=0;

    for(it=post_map.begin();it!=post_map.end();++it)
    {
        double temp_pro=it->second*information_gain[it->first];
        eig+=temp_pro;
    }

    return eig;


}