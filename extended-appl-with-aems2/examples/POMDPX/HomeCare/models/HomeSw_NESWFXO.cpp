#include "simpleHomeSW.h"


using namespace std;

void rewardFu(ofstream& pomdpxfile, int nrSW, string reward);

void observationFu(ofstream& pomdpxfile, int nrSW, int row, int col, int nrDist, bool fullyObsSW, int obstacle);

void stateTrFu(ofstream& pomdpxfile, int row, int col, int nrSW, int nrActions, int obstacle, int actionMap[][7], char *actionVec);

void initialBeliefState(ofstream& pomdpxfile, int row, int col, int nrSW, int obstacle);

void variableGeneration(ofstream& pomdpxfile, string actions,string sw_states, string  sw_observation, string reward, string fullyObsSW, string fullyObsPoz, int obstacl, int row, int col, int nrSW, float discount, string swObservability);

void closeModel(ofstream& pomdpxfile);

void introGeneration(ofstream& pomdpxfile);


int map[100][100];			// map with obstacles and sw
float swOnBelief[100];		// initial BS for SW
float distV[100];			// vector for f(distance)= obs probaility
float mapInit[100][100];	// initial BS for pozition
int SWcoord[2][20];			// coordinates of SW (x, y)
float revardMoving = 0.0, revardStaying = 0.0, revardTurnOn = -1.0, revardTurnOff = 1.0;

unsigned binaryMax(unsigned a)
{
	unsigned c = 0;
	
	for (int i=0; i<a; i++)
	{		
		c <<=1;
		c++;		
	}
	
	return c;
}

int returnInitialUnObsSt(char * file)
{
	int p = 0, nr = 0;
	unsigned bit=0;
	
	fstream initFile;
	initFile.open(file, std::ios_base::in);
	if (initFile.is_open()) 
	{
		initFile >> nr;
		for (int i = 0; i<nr; i++)
		{
				initFile >> bit; 
				p<<=1;
				p= p + bit;
		}
	}
	else
		return -1;
	return p;
}


int main( int argc,  char** argv)
{ 
	//char fxxx[] = "in.txt";
	//cout << "test in.txt " << returnInitialUnObsSt(fxxx) << endl;
		
	string actions = "N E S W F X O", sw_states ="on  off", sw_observation = "O_on O_off unObs", reward = "reward_robot";
	string trueS = "true", falseS = "false", swObservability;

	const int obstacle = 7, switch1 = 2, nrActions = 7;
	int actionMap[2][7] = {	{-1, 0, 1, 0, 0, 0, 0},
							{0, 1, 0, -1, 0, 0, 0}}; // a = [row/column][N E S W f = flip Switch x= stay] order mathers
	char actionVec[] = "NESWFXO";							
	float robotInitPoz = 1.0;	
	int row = 5, col = 5, ObsR, ObsC, nrObs, nrSW, initR, initC;
	float discount  = 0.0;
	bool fullyObsSW = true, fullyObsPoz = true;
	int nrDist = 0;
	
	ofstream modelFile ("model.pomdpx");
	introGeneration( modelFile );

		
	if (argc > 1)
	{
		cout << argv[1];
		fstream modelInput;
				
		modelInput.open(argv[1], std::ios_base::in);
		if (modelInput.is_open()) 
		{

				

			
			cout << " is opend";
			modelInput >> row >> col >> nrObs;
			row++;
			col++;
			for (int i = 0; i <= row; i++)
				for(int j = 0; j <= col; j++)
					if ((i < 1)||(i == row)||(j < 1)||(j == col))			
						map[i][j]=obstacle;			
					else
						map[i][j]=0;
			for (int i = 0; i< nrObs; i++)
			{
				modelInput >> ObsR >>  ObsC;				
				map[ObsR][ObsC] = obstacle;
			}
			modelInput >> nrSW;
			for (int i = 0; i< nrSW; i++)
			{
				modelInput >> ObsR >> ObsC;				
				map[ObsR][ObsC] = switch1;
				SWcoord[0][i] = ObsR;
				SWcoord[1][i] = ObsC;
			}
			modelInput >> discount >> fullyObsPoz >> initR >> initC;
			if (initR<0 && initC<0 )
				robotInitPoz=robotInitPoz/((float)(((row-1)*(col-1))-nrObs));
				
			for (int i=1; i<row; i++)
				for (int j = 1; j<col; j++)						
					if (map[i][j]!=8)
					{		
						if (robotInitPoz != 1.0)
							mapInit[i][j] = robotInitPoz;
						else 
							if (i==initR && j==initC)
								mapInit[i][j] = 1.0;
							else
								mapInit[i][j] = 0.0;
					}
			modelInput >> fullyObsSW;
			if (!fullyObsSW)
			{
				for (int i= 0; i<nrSW; i++)
					 swOnBelief[i] =0.5;
					 
				modelInput >> nrDist;
				for (int i=0; i<nrDist; i++)
					modelInput >> distV[i];
					swObservability = falseS;
			}
			else
			{
				for (int i= 0; i<nrSW; i++)
				{
					modelInput >> swOnBelief[i];
					swObservability =trueS;
				}
			}
			modelInput >> revardTurnOn;				
			modelInput >> revardTurnOff;				
			modelInput >> revardMoving;
			modelInput >> revardStaying;
			modelInput >> revardTurnOn;				
		}
		
	
	}		
	else
	{
		cout << "give map size raw & col (default 5x5)";
		cin >> row;
		cin >> col;
	

		row++;
		col++;

		for (int i = 0; i <= row; i++)
			for(int j = 0; j <= col; j++)
				if ((i < 1)||(i == row)||(j < 1)||(j == col))			
					map[i][j]=obstacle;			
				else
					map[i][j]=0;
		
		
		cout << "\n get nr of obstacles " ;
		cin >>  nrObs;
		cout << " give coord: ";	
	
		for (int i = 0; i< nrObs; i++)
		{
			cin >> ObsR >>  ObsC;
			cout << endl;
			map[ObsR][ObsC] = obstacle;
		}
	
		cout << "\n get nr of switches " ;
		cin >>  nrSW;
		cout << " give coord: ";	
		for (int i = 0; i< nrSW; i++)
		{
			cin >> ObsR >> ObsC;
			cout << endl;
			map[ObsR][ObsC] = switch1;
			SWcoord[0][i] = ObsR;
			SWcoord[1][i] = ObsC;
		}

		cout << "get discount (float)" ;
		cin >> discount;

		cout << "Pozition is fully observable (1/0) ";
		cin >> fullyObsPoz;	
		cout << "give robot initial position coordinate (if uniform = -1 -1)";
		cin >> initR >> initC;
		if (initR<0 && initC<0 )
				robotInitPoz=robotInitPoz/((float)(((row-1)*(col-1))-nrObs));
				
		for (int i=1; i<row; i++)
			for (int j = 1; j<col; j++)
						
				if (map[i][j]!=8)
				{		
					if (robotInitPoz != 1.0)
						mapInit[i][j] = robotInitPoz;
					else 
						if (i==initR && j==initC)
							mapInit[i][j] = 1.0;
						else
							mapInit[i][j] = 0.0;
				}				

	
		
		cout << "switchis are fully observable (1/0) if 0 than initialBS is uniform";
		cin >> fullyObsSW;
	
		if (!fullyObsSW)
			for (int i= 0; i<nrSW; i++)
				 swOnBelief[i] =0.5;
		else
		{
			for (int i= 0; i<nrSW; i++)
			{
				cout<< "give belief of ON for SW" <<i;
				cin >> swOnBelief[i];
			}
		}
	
	}

	cout << "map: \n";		// print map in terminal
	for (int i = 0; i <= row; i++)
	{
		for(int j = 0; j <= col; j++)
		{
			cout << map[i][j] << "  ";
		}
		cout << endl;
	}

	
	variableGeneration(modelFile, actions, sw_states, sw_observation, reward, "true", "true", obstacle, row, col, nrSW, discount, swObservability);

	initialBeliefState(modelFile, row, col, nrSW, obstacle);

	stateTrFu(modelFile, row, col, nrSW, nrActions, obstacle, actionMap, actionVec);
	
	observationFu(modelFile, nrSW, row, col, nrDist, fullyObsSW, obstacle);
	
	rewardFu(modelFile, nrSW, reward);
		
	closeModel( modelFile );

	modelFile.flush();
	modelFile.close();
	
	return 0;
}

void rewardFu(ofstream& pomdpxfile, int nrSW, string reward)
{
	pomdpxfile <<"<RewardFunction> \n \
	<Func> \n \
	<Var>" << reward <<"</Var> \n \
	<Parent>action_robot robot_0";
	
	for (int i=0; i<nrSW; i++)
		pomdpxfile << " sw" << i << "_0";
		
	pomdpxfile << "</Parent> \n \
	<Parameter type=\"TBL\"> \n \
		<Entry> \n \
		<Instance>* *";
		
		for (int i=0; i<nrSW; i++)
			pomdpxfile << " *";
	/// should we give negative error for any action or 0
		pomdpxfile <<"</Instance> \n\
		<ValueTable>"<< revardMoving <<"</ValueTable> \n\
	</Entry> \n";
	
	/// no negative reward for stay action = X
	 	pomdpxfile << "\
	 <Entry> \n \
		<Instance>X *";
		
		for (int i=0; i<nrSW; i++)
			pomdpxfile << " *";
			
		pomdpxfile <<"</Instance> \n\
		<ValueTable>"<< revardStaying <<"</ValueTable> \n\
	</Entry> \n\ ";
	
	
		
	for (int i=0; i<nrSW; i++)
	{
		pomdpxfile <<"\
	<Entry> \n \
		<Instance>F "<< SWcoord[0][i] << "_" << SWcoord[1][i];
		for (int j=0; j<nrSW; j++)
			if (j==i)
				pomdpxfile << " on";
			else
				pomdpxfile << " *";
		pomdpxfile <<"</Instance> \n\
		<ValueTable>"<< revardTurnOn <<"</ValueTable> \n\
	</Entry> \n";
		pomdpxfile <<"\
	<Entry> \n \
		<Instance>F "<< SWcoord[0][i] << "_" << SWcoord[1][i];
		for (int j=0; j<nrSW; j++)
			if (j==i)
				pomdpxfile << " off";
			else
				pomdpxfile << " *";
		pomdpxfile <<"</Instance> \n\
		<ValueTable>"<< revardTurnOff <<"</ValueTable> \n\
	</Entry> \n";	
	}
	 
	pomdpxfile <<"\n \
		</Parameter> \n\
	</Func> \n\
 </RewardFunction> \n \n";
}

void observationFu(ofstream& pomdpxfile, int nrSW, int row, int col, int nrDist, bool fullyObsSW, int obstacle)
{
	string obsStr[2]= {"on", "off"};
	int distance = 0;
	float prob = 0.0;
	pomdpxfile << "<ObsFunction>\n";
	float pV[nrSW];
	
	
	if (!fullyObsSW)
	{
		for (int swID = 0; swID < nrSW; swID++)
		{
			pomdpxfile << "\
		<CondProb> \n \
		<Var>obs_sw"<<swID<<"</Var> \n \
		<Parent> action_robot robot_1";
		for (int i = 0; i < nrSW; i++)
			pomdpxfile << " sw" << i << "_1";
		
			pomdpxfile << "</Parent> \n \
		<Parameter type=\"TBL\"> \n";
		
			pomdpxfile << "\
			<Entry>\n \
				<Instance>* *";
			for (int i=0; i<nrSW; i++)
				pomdpxfile << " *";
			pomdpxfile << " -</Instance> \n\
			 	<ProbTable> 0.0 0.0 1.0</ProbTable>\n\
		 	</Entry>\n\n";
		 	
		 	
			for( int i=1; i<row; i++)			{	
					
				for(int j=1; j<col ; j++)
				{
					distance = max(abs(SWcoord[0][swID]-i), abs(SWcoord[1][swID]-j));
					if (map[i][j] != obstacle && distance < nrDist)
					{	
						/*
						unsigned nrOfInfluences = 0;
						for (int s=0; s<nrSW; s++)
						{
							distance = max(abs(SWcoord[0][s]-i), abs(SWcoord[1][s]-j));
							if (distance >= nrDist)
							{
								pV[s] = -1;
							}
							else
							{
								nrOfInfluences++;
								pV[s] =	distV[distance];
							}
							
						}
						
						unsigned tmp, maxNr, bit; // nr of max bits obs infuencing each other
						maxNr = binaryMax(nrOfInfluences);
//						cout << c << endl;
						for (int i2=0; i2 <= maxNr; i2++)
						{
							tmp= i2;
//							cout << t << ": ";
							pomdpxfile << "\
			<Entry>\n \
				<Instance>* "<< i<<"_"<<j;
							
							float probVal = 1.0;
							cout <<endl <<i << "_" << j <<" C: " << tmp << " - ";
							
							for (int i3=0; i3<nrSW; i3++)
								if (pV[i3] < 0)
								{
									pomdpxfile << " *"; // does not matter state of sw
								}
								else
								{
									bit = (tmp & 1);								
									tmp>>=1;	
									pomdpxfile << " " << obsStr[bit]; // state of switch
									probVal = probVal *abs(bit-pV[i3]);
									cout << pV[i3] << " ";
								}
							pomdpxfile << " -</Instance>\n \
							<ProbTable>" << probVal <<" "<< 1-probVal << " 0.0</ProbTable>\n\
		 	</Entry>\n\n";
								
						}
					*/
					// if no correlation needed between switches
					
						prob = distV[distance];
						pomdpxfile << "\
			<Entry>\n \
				<Instance>O "<< i << "_" << j;
						for (int k=0; k<nrSW; k++)
							if (k == swID)
								pomdpxfile << " -";
							else
								pomdpxfile << " *";
						
						pomdpxfile << " -</Instance>\n \
			 	<ProbTable>"<< prob << " " << 1-prob << " 0 " << 1-prob << " " << prob << " 0</ProbTable>\n\
		 	</Entry>\n\n";
						
					}
				}
			}
			pomdpxfile << "\
		</Parameter>\n\
	</CondProb>\n";
		}
	}	
	else
	{ // revrite later for compability with MDP
		for (int i=0; i<nrSW; i++)
			pomdpxfile << " sw"<< i <<"_1";
		pomdpxfile <<"</Parent> \n \
		<Parameter type=\"TBL\">\n";
		
			for (int i=0; i<nrSW; i++)
			{
				pomdpxfile <<"\
				<Entry> \n \
 				<Instance>* *";
 				for (int j=0; j<nrSW; j++)
 					if (j==i)
 						pomdpxfile << " -";
					else
 						pomdpxfile << " *";
 						
				pomdpxfile <<" -</Instance> \n \
  				<ProbTable>1.0 0.0 0.0 0.0 1.0 0.0</ProbTable>\n \
				</Entry> \n";
			}
		pomdpxfile << "\
		</Parameter>\n\
	</CondProb>\n ";
	}
	
	pomdpxfile <<"</ObsFunction>\n\n";
}

void stateTrFu(ofstream& pomdpxfile, int row, int col, int nrSW, int nrActions, int obstacle, int actionMap[][7], char *actionVec)
{
	int iV = 0, jV = 0;
	string go = "1";
	pomdpxfile << "<StateTransitionFunction>\n \
	<CondProb> \n \
		<Var>robot_1</Var> \n \
		<Parent> action_robot robot_0</Parent> \n \
		<Parameter type=\"TBL\">\n ";
			/*<Entry> \n\
 				<Instance>* * *</Instance> \n\
 				<ProbTable>0</ProbTable>\n\
 			</Entry>\n";*/
		for (int i=1; i<row; i++)	
			for(int j=1; j<col; j++)
				if (map[i][j]!=obstacle)
				{					
					for (int a=0; a<nrActions; a++)
					{	
						iV = i+actionMap[0][a];
						jV = j+actionMap[1][a];
						
							if (map[iV][jV]!=obstacle )
								pomdpxfile << "\
			<Entry> \n \
				<Instance>" <<actionVec[a] <<" "<<i<<"_"<<j<<" "<< iV <<"_"<< jV <<"</Instance> \n \
				<ProbTable>1</ProbTable>\n \
			</Entry> \n\n";
							else
								pomdpxfile << "\
			<Entry> \n \
				<Instance>" <<actionVec[a] <<" "<<i<<"_"<<j<<" "<< i <<"_"<< j <<"</Instance> \n \
				<ProbTable>1</ProbTable>\n \
			</Entry> \n\n";
						
					}
				}
				
	pomdpxfile <<"\
			</Parameter> \n \
	</CondProb>\n\n"; 
	
	for (int i=0; i<nrSW; i++)
	{
		pomdpxfile << "\
	<CondProb> \n \
		<Var>sw"<< i <<"_1</Var> \n \
		<Parent> action_robot robot_0 sw"<< i <<"_0</Parent> \n \
		<Parameter type=\"TBL\">\n\
			<Entry> \n\
			<Instance>* * - -</Instance> \n \
			<ProbTable>1 0 0 1</ProbTable> \n \
			</Entry>	\n\
			<Entry> \n\
 				<Instance>F "<<SWcoord[0][i]<<"_"<<SWcoord[1][i]<<" - -</Instance> \n \
 				<ProbTable>0 1 1 0</ProbTable> \n \
			</Entry>	\n\
		</Parameter> \n \
	</CondProb>\n\n";
	
	}
	pomdpxfile << "</StateTransitionFunction>\n \n";	
}

void initialBeliefState(ofstream& pomdpxfile, int row, int col, int nrSW, int obstacle)
{
	pomdpxfile << "<InitialStateBelief> \n \
	<CondProb> \n \
		<Var>robot_0</Var> \n \
		<Parent>null</Parent>\n\
		<Parameter type = \"TBL\"> \n \
			<Entry> \n\
				<Instance>-</Instance> \n\
					<ProbTable>";
	for (int i = 1; i < row; i++ )
		for (int j=1; j<col; j++)
			if (map[i][j] != obstacle)
				pomdpxfile << mapInit[i][j]<< " ";
	pomdpxfile << "</ProbTable> \n \
			</Entry> \n \
		</Parameter>\n \
	</CondProb>\n\n";

	for (int i=0; i<nrSW; i++)
	pomdpxfile << "\
	<CondProb> \n \
		<Var>sw"<<i<<"_0</Var>\n \
		<Parent>null</Parent> \n \
		<Parameter type = \"TBL\">\n \
			<Entry>\n \
				<Instance>-</Instance>\n \
					<ProbTable>" << swOnBelief[i] <<" "<< (1.0-swOnBelief[i]) << "</ProbTable> \n \
			</Entry> \n \
		</Parameter> \n \
	</CondProb>\n\n";
	
	pomdpxfile << "</InitialStateBelief>\n\n";	
}


void variableGeneration(ofstream& pomdpxfile, string actions,string sw_states, string  sw_observation, string reward, string fullyObsSW, string fullyObsPoz, int obstacl, int row, int col, int nrSW, float discount, string swObservability)
{
	pomdpxfile << "<Discount>"<< discount <<"</Discount> \n\n <Variable> \n\
	<StateVar vnamePrev=\"robot_0\" vnameCurr= \"robot_1\" fullyObs= \"" << fullyObsPoz << "\"> \n\
		<ValueEnum>";

	for (int i =1; i < row; i++)
		for (int j =1; j<col; j++)
			if (map[i][j] < obstacl)
			pomdpxfile << i << "_" << j << " ";	

	pomdpxfile << "</ValueEnum> \n\
 	</StateVar> \n \n";
	for (int i= 0; i < nrSW; i++)
	pomdpxfile << "\
	<StateVar vnamePrev=\"sw" << i << "_0\" vnameCurr= \"sw" << i << "_1\" fullyObs=\"" << swObservability << "\"> \n \
		<ValueEnum>" << sw_states <<"</ValueEnum>\n </StateVar> \n \n";
	
	for (int i= 0; i < nrSW; i++)
	pomdpxfile << "\
	<ObsVar vname=\"obs_sw"<< i  <<"\">\n \
		<ValueEnum>" << sw_observation << "</ValueEnum>\n \
	</ObsVar>\n \n ";
	
	pomdpxfile << "\
	<ActionVar vname=\"action_robot\">\n \
		<ValueEnum>" << actions << "</ValueEnum> \n \
	</ActionVar> \n \n \
	<RewardVar vname=\""<< reward << "\"/>\n \n </Variable>\n\n ";
}


void introGeneration(ofstream& pomdpxfile)
{
    pomdpxfile << "<?xml version='1.0' encoding='ISO-8859-1'?>\n \ \n\ \n\
	<pomdpx version='0.1' id='autogenerated' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:noNamespaceSchemaLocation='pomdpx.xsd'>\n\ \n\ ";
}


void closeModel(ofstream& pomdpxfile)
{
	pomdpxfile << "</pomdpx>";
}


