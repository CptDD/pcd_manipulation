#include <despot/core/belief.h>
#include <despot/core/pomdp.h>
#include <despot/core/information.h>
#include <cmath>
#include <iostream>

using namespace std;

namespace despot {

Information::Information(DSPOMDP* m):model(m){}

void Information::show()
{
    cout<<"Show show!"<<endl;
    cout<<pre->text()<<endl;
    cout<<post->text()<<endl;
}

void Information::add_pre(Belief *b)
{
    pre=b->MakeCopy();
}

void Information::add_post(Belief *b)
{
    post=b->MakeCopy();
}

void Information::compute_information_gain()
{
    map<string,double> pre_pdf=pre->get_pdf();
    map<string,double> post_pdf=post->get_pdf();

    vector<pair<string, double> > pre_pairs = SortByValue(pre_pdf);
    vector<pair<string, double> > post_pairs = SortByValue(post_pdf);

    vector<double> information_gain;

    for(int i=0;i<post_pairs.size();i++)
    {
       pair<string,double> pair=pre_pairs[i];
    }

    for (int i = 0; i < pre_pairs.size(); i++) {
		pair<string, double> pair = pre_pairs[i];
	    cout<<pair.first << endl;
        
	}

     for (int i = 0; i < post_pairs.size(); i++) {
		pair<string, double> pair = post_pairs[i];
        cout<<pair.first<<endl;
	}

    if(pre_pairs.size()==post_pairs.size())
    {
        for (int i = 0; i < post_pairs.size(); i++) {
		pair<string, double> post_pair = post_pairs[i];
        pair<string, double> pre_pair = pre_pairs[i];

        double temp_div=post_pair.second/pre_pair.second;
        double temp_val=post_pair.second*log2(temp_div);

        cout<<"Temp val :"<<temp_val<<endl;

        information_gain.push_back(temp_val);

        }
    }


    
  
}

} // namespace despot
