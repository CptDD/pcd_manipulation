
PWD_L=../../..
HC=examples/POMDPX/HomeCare
AUV=examples/POMDPX/AUVnavigation
RS=examples/POMDPX/RockSample
	
### RUN Home Care OPMDP
bash $PWD_L/$HC/runOPMDPSimulation.sh 

### RUN Home Care SARSOP
bash $PWD_L/$HC/runSARSOPSimulation.sh

### RUN Rock Sample OPMDP
#bash $PWD_L/$RS/runOPMDPSimulationRock7_8.sh

### RUN Rock Sample SARSOP
#bash $PWD_L/$RS/runSARSOPSimulationRock7_8.sh

### RUN AUV OPMDP
#bash $PWD_L/$AUV/runOPMDPSimulateAUV.sh

### RUN AUV SARSOP
#bash $PWD_L/$AUV/runSARSOPSimulationAUV.sh	
