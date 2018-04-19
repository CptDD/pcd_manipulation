#!/bin/bash
PWD_L=../../..

MODEL_PATH=$PWD_L/examples/POMDPX/RockSample
SIMULATOR_PATH=$PWD_L/src
MODEL=$PWD_L/examples/POMDPX/RockSample_7_8.pomdpx
INPUT=$PWD_L/examples/POMDPX/RockSample/initYRock.txt

SIMULATION_OUTPUT_FILE=simOut_SARSOP_HC5x5.txt
OUTPUT=out_SARSOP_HC.policy
GRAPH=graph_SARSOP_HC5x5.dot

#$MODEL_PATH/makeModel $MODEL_PATH/modelIn5x5.txt
#mv model.pomdpx $MODEL

### startsimulation
NRSIM=2
STARTSIM=1

for (( i=$STARTSIM; i<=$NRSIM; i++ ))
do
	echo "============ At $i evaluation ================="
	$SIMULATOR_PATH/pomdpsol $MODEL --output $OUTPUT$i -p 1e-$i 
done		



for (( i=$STARTSIM; i<=$NRSIM; i++ ))
do
	echo "============ At $i simulation ================="
	$SIMULATOR_PATH/pomdpsim  --simLen 30 --simNum 100 --policy-file $OUTPUT$i $MODEL --output-file $SIMULATION_OUTPUT_FILE$i  --UnObsInitStateFile $INPUT

done		


#$SIMULATOR_PATH/pomdpsol $MODEL --output $OUTPUT
#$SIMULATOR_PATH/pomdpsim  --simLen 30 --simNum 1 --policy-file $OUTPUT $MODEL --output-file $SIMULATION_OUTPUT_FILE  --UnObsInitStateFile $INPUT
#$SIMULATOR_PATH/polgraph --policy-file $OUTPUT --policy-graph $GRAPH $MODEL

#cp $MODEL_PATH/initY.txt $INPUT
#$MODEL_PATH/simulate $MODEL_PATH/simpleHomeSwitch.png $SIMULATION_OUTPUT_FILE 5

