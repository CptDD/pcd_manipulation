#include <despot/core/belief.h>
#include <despot/core/pomdp.h>
#include <despot/core/information.h>
#include <cmath>
#include <iostream>
#include <string>

using namespace std;

namespace despot {

void Information::show(map<string,double> values)
{
    map<string,double>::iterator it;

    for(it=values.begin();it!=values.end();++it)
    {
        cout<<it->first<<" "<<it->second<<endl;
    }
    
}

double Information::get_max(Belief *b)
{
    double max_val=0.0;


	/*double max=0.0; 

	map<string,double> pdf;
	for(int i=0;i<particles_.size();i++)
	{
		pdf[particles_[i]->text()]+=particles_[i]->weight;
	}

	vector<pair<string, double> > pairs = SortByValue(pdf);

	cout<<pairs[0].first<<" "<<pairs[0].second<<endl;*/

    return max_val;
}

void Information::add_pre(Belief *b)
{
    pre=b->MakeCopy();
}

void Information::add_post(Belief *b)
{
    post=b->MakeCopy();
}

map<string,double> Information::pdf_to_tiger(map<string,double> pdf)
{
    map<string,double> out;
    
    map<string,double>::iterator it;

    return out;
}

map<string,double> Information::pdf_extended_to_map(map<string,double> pdf)
{
    map<string,double> out;

    map<string,double>::iterator it;

    for(it=pdf.begin();it!=pdf.end();++it)
    {
        pair<string,double> temp=*it;

        int pos=-1;

        pos=temp.first.find("state_1:");

        if(pos!=-1)
        {
            int pos2=-1;
            pos+=8;
            pos2=temp.first.find("state_3:");
            string parsed_value=temp.first.substr(pos,pos2-pos-2); //-2 for eliminating the comma(,)

            out[parsed_value]=temp.second;
        }
    }

    if(pdf.size()!=this->class_numbers)
    {
        it=out.find("elongated");

        if(it==out.end())
        {
            out["elongated"]=0;
        }

        it=out.find("livarno");

        if(it==out.end())
        {
            out["livarno"]=0;
        }

        it=out.find("mushroom");

        if(it==out.end())
        {
            out["mushroom"]=0;
        }

        it=out.find("standard");

        if(it==out.end())
        {
            out["standard"]=0;
        }
    }

    return out;
}

map<string,double> Information::pdf_to_map(map<string,double> pdf)
{
    map<string,double> out;

    map<string,double>::iterator it;

    for(it=pdf.begin();it!=pdf.end();++it)
    {
        pair<string,double> temp=*it;

        int pos=-1;

        //cout<<temp.first<<endl;
        pos=temp.first.find("state_1:");

        if(pos!=-1)
        {
            int pos2=-1;
            pos+=8; //offset to the name of the state variable
            pos2=temp.first.find("]"); //eliminated the final ']'
            string parsed_value=temp.first.substr(pos,pos2-pos);
        
            out[parsed_value]=temp.second;
        }

    }

    if(pdf.size()!=this->class_numbers)
    {
        it=out.find("elongated");

        if(it==out.end())
        {
            out["elongated"]=0;
        }

        it=out.find("livarno");

        if(it==out.end())
        {
            out["livarno"]=0;
        }

        it=out.find("mushroom");

        if(it==out.end())
        {
            out["mushroom"]=0;
        }

        it=out.find("standard");

        if(it==out.end())
        {
            out["standard"]=0;
        }
    }


    return out;
}


double Information::compute_entropy()
{
    /*entropy computation*/

    return 0.0;
}

double Information::compute_expected_information_gain()
{
    map<string,double> post_map=this->pdf_to_map(post->get_pdf());
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

double Information::compute_extended_expected_information_gain()
{
    double eig=0;

    map<string,double> post_map=this->pdf_extended_to_map(post->get_pdf());
    map<string,double> information_gain=this->compute_extended_information_gain();

    map<string,double>::iterator it;

    for(it=post_map.begin();it!=post_map.end();++it)
    {
        double temp_pro=it->second*information_gain[it->first];
        eig+=temp_pro;
    }
    

    return eig;
}



map<string,double> Information::compute_information_gain()
{
    map<string,double> pre_pdf=pre->get_pdf();
    map<string,double> post_pdf=post->get_pdf();

    map<string,double> pre_map,post_map,information_gain;
    map<string,double>::iterator it;

    //cout<<post_pdf.size()<<endl;

    pre_map=this->pdf_to_map(pre_pdf);
    post_map=this->pdf_to_map(post_pdf);

    /*cout<<"Pre"<<endl;
    this->show(pre_map);
    cout<<"Post"<<endl;
    this->show(post_map);*/

    for(it=pre_map.begin();it!=pre_map.end();++it)
    {
        //cout<<it->first<<endl;
        double pre_val=it->second;
        double post_val=post_map.at(it->first);



        if(pre_val!=0 && post_val!=0)
        {
            double temp_div=post_val/pre_val;
            double temp_val=post_val*log2(temp_div);

            information_gain[it->first]=temp_val;
        }

        if(pre_val==0||post_val==0)
        {
            information_gain[it->first]=0;
        }

        /*if(pre_val==0)
        {
            information_gain[it->first]=0.5;
        }

        if(post_val==0)
        {
            information_gain[it->first]=0;
        }*/
    }

    /*cout<<"===Information gain==="<<endl;
    for(it=information_gain.begin();it!=information_gain.end();++it)
    {
        cout<<it->first<<" "<<it->second<<endl;
    }
    cout<<"======================"<<endl;*/

    return information_gain;

    /*vector<pair<string, double> > pre_pairs = SortByValue(pre_pdf);
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



        }*/ 
}

map<string,double> Information::compute_extended_information_gain()
{
    map<string,double> pre_pdf=pre->get_pdf();
    map<string,double> post_pdf=post->get_pdf();

    map<string,double> pre_map,post_map,information_gain;
    map<string,double>::iterator it;

    //cout<<post_pdf.size()<<endl;

    pre_map=this->pdf_extended_to_map(pre_pdf);
    post_map=this->pdf_extended_to_map(post_pdf);

    /*cout<<"Pre"<<endl;
    this->show(pre_map);
    cout<<"Post"<<endl;
    this->show(post_map);*/

    for(it=pre_map.begin();it!=pre_map.end();++it)
    {
        //cout<<it->first<<endl;
        double pre_val=it->second;
        double post_val=post_map.at(it->first);



        if(pre_val!=0 && post_val!=0)
        {
            double temp_div=post_val/pre_val;
            double temp_val=post_val*log2(temp_div);

            information_gain[it->first]=temp_val;
        }

        if(pre_val==0||post_val==0)
        {
            information_gain[it->first]=0;
        }

        /*if(pre_val==0)
        {
            information_gain[it->first]=0.5;
        }

        if(post_val==0)
        {
            information_gain[it->first]=0;
        }*/
    }

    /*cout<<"===Information gain==="<<endl;
    for(it=information_gain.begin();it!=information_gain.end();++it)
    {
        cout<<it->first<<" "<<it->second<<endl;
    }
    cout<<"======================"<<endl;*/

    return information_gain;

    /*vector<pair<string, double> > pre_pairs = SortByValue(pre_pdf);
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



        }*/ 
}

} // namespace despot
