#!/bin/bash
PWD_L=../../..

MODEL_PATH=$PWD_L/examples/POMDPX/HomeCare/models
SIMULATOR_PATH=$PWD_L/src
MODEL=$PWD_L/examples/POMDPX/HomeCare/model5x9.pomdpx
INPUT=$PWD_L/examples/POMDPX/HomeCare/init5x9.txt

SIMULATION_OUTPUT_FILE=simOut_SARSOP.txt
OUTPUT=out_SARSOP.policy
GRAPH=graph_SARSOP.dot

#$MODEL_PATH/makeModel $MODEL_PATH/modelIn5x5.txt
#mv model.pomdpx $MODEL


NRSIM=30
STARTSIM=1

#rm $PLOT
rm $SIMULATION_OUTPUT_FILE

### startsimulation

for (( i=$STARTSIM; i<=$NRSIM; i++ ))
do
	echo "============ At $i evaluation ================="
	$SIMULATOR_PATH/pomdpsol $MODEL --output $OUTPUT$i -p 1e-4 >> solverSarsop_timeTest
done		



#for (( i=$STARTSIM; i<=$NRSIM; i++ ))
#do
#	echo "============ At $i simulation ================="
#	$SIMULATOR_PATH/pomdpsim  --simLen 30 --simNum 100 --policy-file $OUTPUT$i $MODEL --output-file $SIMULATION_OUTPUT_FILE$i  --UnObsInitStateFile $INPUT >> simulatorSarsop

#done		

#$SIMULATOR_PATH/polgraph --policy-file $OUTPUT --policy-graph $GRAPH $MODEL

#cp $MODEL_PATH/initY.txt $INPUT
#models/simulate models/simpleHomeSwitch.png $SIMULATION_OUTPUT_FILE$i 5
#ls

