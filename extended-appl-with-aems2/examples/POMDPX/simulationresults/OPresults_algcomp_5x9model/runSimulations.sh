
PWD_L=../../..
HC=examples/POMDPX/HomeCare
AUV=examples/POMDPX/AUVnavigation
RS=examples/POMDPX/RockSample

### only model5x9 with R(*,'stay')=0.6
	
### RUN Home Care OPMDP AEMS2
bash $PWD_L/$HC/runOPMDPSimulationAEMS2.sh 

### RUN Home Care OPMDP FHHOP
bash $PWD_L/$HC/runOPMDPSimulationFHHOP.sh 

### RUN Home Care OPMDP DHS
bash $PWD_L/$HC/runOPMDPSimulationDHS.sh 

### RUN Home Care OPMDP DESPOT
#bash $PWD_L/$HC/runOPMDPSimulation.sh 

### RUN Home Care SARSOP
bash $PWD_L/$HC/runSARSOPSimulation.sh

