// OnlineOPsolver.cpp : Defines the entry point for the console application.
//


#ifdef _WIN32
#include <windows.h> // Windws insludes
#include "stdafx.h" //  Windows stuff
#endif

#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <cfloat>

#ifdef _MSC_VER
#include "getopt.h"
#define NOMINMAX 


#else
#include <getopt.h>
#include <sys/time.h>
#endif

#include <signal.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <assert.h>
#include <cstring>
#include <cstdlib>


#include "CPTimer.h"

#include "GlobalResource.h"
//#include "ActionSelector.h"
//#include "PolicyFollower.h"

#include "solverUtils.h"
#include "Parser.h"
#include "POMDP.h"
#include "ParserSelector.h"
#include "MOMDP.h"
#include "SARSOP.h"

#include "OPMDP.h"
#include "BackupAlphaPlaneMOMDP.h"
#include "BackupBeliefValuePairMOMDP.h"
#include "dataLogger.h"

//#include "FSVI.h"
//#include "GES.h"
#include "FullObsUBInitializer.h"
#include "FastInfUBInitializer.h"

// visualisation libraries		PAEL
//#include "opencv\highgui.h"
//#include "opencv\cv.h"
/* // Windws insludes
#include "opencv2\core.hpp"
#include "opencv2\highgui.hpp"
#include "opencv2\imgproc.hpp"
*/
#ifdef OPENCV
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui_c.h"
using namespace cv;
#endif

#define TREETEST_OFF

using namespace std;

using namespace momdp;

#undef DEBUG_TRACE_ON 
#define REWARD_TESTING_OFF

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#include <ctime>
#endif

/* Remove if already defined */
typedef long long int64; typedef unsigned long long uint64;

/* Returns the amount of milliseconds elapsed since the UNIX epoch. Works on both
 * windows and linux. */

//uint64 GetTimeMs64_lib()
//{
//#ifdef _WIN32
// /* Windows */
// FILETIME ft;
// LARGE_INTEGER li;
//
// /* Get the amount of 100 nano seconds intervals elapsed since January 1, 1601 (UTC) and copy it
//  * to a LARGE_INTEGER structure. */
// GetSystemTimeAsFileTime(&ft);
// li.LowPart = ft.dwLowDateTime;
// li.HighPart = ft.dwHighDateTime;
//
// uint64 ret = li.QuadPart;
// ret -= 116444736000000000LL; /* Convert from file time to UNIX epoch time. */
// ret /= 10000; /* From 100 nano seconds (10^-7) to 1 millisecond (10^-3) intervals */
//
// return ret;
//#else
// /* Linux */
// struct timeval tv;
//
// gettimeofday(&tv, NULL);
//
// uint64 ret = tv.tv_usec;
// /* Convert from micro seconds (10^-6) to milliseconds (10^-3) */
// ret /= 1000;
//
// /* Adds the seconds (10^0) after converting them to milliseconds (10^-3) */
// ret += (tv.tv_sec * 1000);
//
// return ret;
//#endif
//}

#ifdef __cplusplus
extern "C"
#endif
{
	extern unsigned long GlobalMemLimit;
}

//struct OutputParams_lib {
//	double timeoutSeconds;
//	double interval;
//	OutputParams_lib(void);
//};
//
//OutputParams_lib::OutputParams_lib(void) {
//	timeoutSeconds = -1;
//	interval = -1;
//}


class POMDPplanner
{
public:
	int a;
	explicit POMDPplanner( char* model, int budget, bool simulate, bool visualise, int simLength, int simTime,  char* outputFile,  char* UnObsInitStateFile);
	~POMDPplanner(void);

	int nextIteration(int obseravtion, int position);

private:
	int currentAction, newUnobsState, currObservation;

	SolverParams* p; // = &GlobalResource::getInstance()->solverParams;
	//OutputParams_lib op;
	string baseName;
	dataLogger dataLog;

	PointBasedAlgorithm* solver;

	OPMDP* opSolver; // = NULL;
	BackupAlphaPlaneMOMDP* lbBackup; // = new BackupAlphaPlaneMOMDP();
	BackupBeliefValuePairMOMDP* ubBackup; // = new BackupBeliefValuePairMOMDP();
	bool simulation, visualization;
	SharedPointer<MOMDP> problem;

	void simulateAll();
	int returnInitialUnObsSt(const char * file);

	// b represents a discrete probability distribution Pr(outcome = i) = b(i).
	// Chooses an outcome according to the distribution.
//	inline int chooseFromDistribution(const DenseVector& b)
//	{
//		double r = unit_rand();
//		FOR(i, b.size())
//		{
//			r -= b(i);
//			if (r <= 0)
//			{
//				return i;
//			}
//		}
//		return b.data.size() - 1;
//	}
};


