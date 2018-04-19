//#include "opencv2/imgcodecs.hpp"
//#include "opencv2/highgui.hpp"
//include <opencv2/imgproc.hpp>
//#include <opencv2/core/utility.hpp>
//#include "opencv2/highgui/highgui_c.h"

//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include <opencv2/opencv.hpp>
//#include <opencv/cv.hpp>
//#include <highgui.h>

#include <stdio.h>
#include "simpleHomeSW.h"

using namespace std;
using namespace cv;

cv::Point getPozition(char *line);
void printBool(bool *b, int nr);

int main(int argc, char** argv )
{	
	
    if ( argc < 4 )
    {
        printf("usage: ./simualte <Image_Path> <Simulation.txt> <simulation time s *0.1>\n");
        return -1;
    }

    Mat image, originalImg;
    image = imread( argv[1], 1 );
	originalImg = image.clone();
	
    int sleepTime = 0;
    if (argc == 4 )
    {
    		sleepTime = atoi( argv[3] )*100;
	}			
			
	
	
    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display Image", CV_WINDOW_AUTOSIZE );
    imshow("Display Image", image);

    
    
    fstream modelInput;				
	modelInput.open(argv[2], std::ios_base::in);
	
	if (modelInput.is_open()) 
	{
			char line[200];
			int i = 0,  nrSW = 0;
			char *poz1, *poz2;
			bool swV[20], stateChanged = false, initialPozition=true, flipedSW = false, staying = false, observe =false;
			
			cv::Point first, next;			
		
			char firstChar;
			int simulationNR = 0;
			char *poz;
			
			while (!modelInput.eof())
			{
				i++;		

				//cout << i << ": line \n";				
				modelInput.getline(line, 200);	
				//fistChar = line[1];
				
				switch (line[0])
				{
				case 'X':
					next = getPozition(line);
					stateChanged = true;
					
					//cout << "X "<< next <<"\n";
					break;
				case 'A':
					poz = strchr(line, '(');	
					if (poz[1] == 'F') flipedSW = true;
					if (poz[1] == 'X') staying = true;
					if (poz[1] == 'O') observe = true;
					//cout << "A \n";
					break;
				case '>':
					initialPozition=true;
					simulationNR = min(200, simulationNR+25);
					image = originalImg.clone();
					//cout << "> \n";
					break;									
				}
				
				
				if (initialPozition && stateChanged)
				{
					initialPozition = false;
					stateChanged = false;
					
					first = next;
					//cout << "init f" << first;
				}
				
				if (!initialPozition)
				{
					
					if (stateChanged)
					{
						stateChanged =false;

	cv::line(image, first, next, Scalar(simulationNR,200-simulationNR,simulationNR),4,8,0);

						imshow("Display Image", image);
						waitKey(sleepTime);
						first = next;
					}
					
					if (flipedSW)
					{
						flipedSW = false;
						circle(image, next, 15, Scalar(200-simulationNR,220,222), 15, 8, 0);
					    imshow("Display Image", image);
						waitKey(sleepTime);
					}
					
					if (staying)
					{
						staying=false;
						putText(image, "X", next -Point(5,-10) , 
    FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(simulationNR,200-simulationNR,simulationNR), 2, CV_AA);
    					imshow("Display Image", image);
    					waitKey(sleepTime);
					}
					if (observe)
					{
						observe=false;
						putText(image, "O", next -Point(-15,-15) , 
    FONT_HERSHEY_COMPLEX_SMALL, 1.2, Scalar(simulationNR,200-simulationNR,simulationNR), 2, CV_AA);
    					imshow("Display Image", image);
  					waitKey(sleepTime);
					}
				}
						
				/*			 	
			 	stateChanged = false;			 	
			 	
				if (i == 2) 
				{
					//cout << "init state:"<< line << endl;					
					
					first = getPozition(line);				
					
					char *poz, *pozLast;
					
					poz = strchr(line, ',');

					while (poz != NULL)
					{
						if (poz[2] == 'n')
							swV[nrSW] = true;
							
						if (poz[2] == 'f')
							swV[nrSW] = false;
						//cout <<"  =======  " << poz[3];						
						poz = strchr(poz+1, ',');						
						nrSW++;
						
					}
					//printBool(swV, nrSW);
					
				}
				else 
					if ((i-7)%6 == 0 && i != 1)
					{
						//cout << "droaw: " << line << endl;
						stateChanged = false;					
						next = getPozition(line);
						bool tmpState = true;
						int j=0;
						
						char *poz;
						poz = strchr(line, ',');
						while (poz != NULL)						{
							
							if (poz[2] == 'n')
								tmpState = true;
							else		
								if (poz[2] == 'f')
									tmpState = false;
							
							if (tmpState != swV[j])
							{
								circle(image, next, 15, Scalar(200,220,222), 15, 8, 0);
								swV[j] = tmpState;
							}
							poz = strchr(poz+1, ',');
							j++;
							
						}
						//cout<< "       ";
						//printBool(swV, nrSW);
					
						arrowedLine(image, first, next, Scalar(0,200,0),4,8,0,0.1);
						imshow("Display Image", image);
						waitKey(sleepTime);
						first = next;
						
					}
				
				if (abs(i-5)%6 == 0)
				{
					char *poz = strchr(line, '(');	
					if (poz[1] == 'F')
						circle(image, next, 15, Scalar(200,220,222), 15, 8, 0);
				}	
				*/	
			}
		putText(image, "Simulations finished.", cvPoint(30,30), 
    FONT_HERSHEY_COMPLEX_SMALL, 1.2, cvScalar(0,0,200), 2, CV_AA);
    	imshow("Display Image", image);
		}
		else
		{
			cout << "\n cant open simulation results .txt \n";
		}
    
	cout << "simualtion done";
    waitKey(0);

    return 0;
}


cv::Point getPozition(char *line)
{
	int x=0, y=0, j = 0;

	//cout << "in function" << line;
	
	cv:Point pozition;
	
	while ( x == 0 )
	{	
		x = atoi(line+j);
		j++;
	}	
	
	while ( y == 0 )
	{	
		y = atoi(line+j);
		j++;
	}
	
	pozition.x = y*100-50;	
	pozition.y = x*100-50;
		
	//cout <<"return poz= " << pozition << endl;
	return pozition;	
}
	
void printBool(bool *b, int nr)
{
	for (int i=0; i<nr; i++)	
		cout << std::boolalpha << b[i] << " ";
	cout << endl;
}

