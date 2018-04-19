#!/bin/bash
PWD_L=../../..

MODEL_PATH=$PWD_L/examples/POMDPX/AUVnavigation
SIMULATOR_PATH=$PWD_L/src
MODEL=$PWD_L/examples/POMDPX/auvNavigation.pomdpx
INPUT=$PWD_L/examples/POMDPX/RockSample/initYAUV.txt

SIMULATION_OUTPUT_FILE=simOut_SARSOP_AUV.txt
OUTPUT=out_SARSOP_AUV.policy
GRAPH=graph_SARSOP_AUV.dot

$SIMULATOR_PATH/pomdpsol $MODEL --output $OUTPUT
$SIMULATOR_PATH/pomdpsim  --simLen 200 --simNum 100 --policy-file $OUTPUT $MODEL --output-file $SIMULATION_OUTPUT_FILE  --UnObsInitStateFile $INPUT
$SIMULATOR_PATH/polgraph --policy-file $OUTPUT --policy-graph $GRAPH $MODEL






