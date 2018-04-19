#!/bin/bash

PWD_L=../../..

MODEL_PATH= $PWD_L/examples/POMDPX/HomeCare/models
SIMULATOR_PATH=$PWD_L/src
MODEL=$PWD_L/examples/POMDPX/HomeCare/model5x9.pomdpx
INPUT=$PWD_L/examples/POMDPX/HomeCare/init5x9.txt

OUTPUT=out_OP_HC1x3.policy
GRAPH=graph_OP_HC1x3.dot
OUTPUT_OP=simOUT_OP_HC1x3.txt
PLOT=performance_OP_HC1x3.png
SIMULATION_OUTPUT_FILE=simulationR.txt

#$MODEL_PATH/makeModel $MODEL_PATH/modelIn5x5.txt
#mv model.pomdpx $MODEL

NRSIM=100
STARTSIM=1

rm $PLOT
rm $SIMULATION_OUTPUT_FILE

### startsimulation
for j in 1 10 100 500 750 1000 #1500
do
	for (( i=$STARTSIM; i<=$NRSIM; i++ ))
	do
		echo "============ At $i simulation for value $j ================="
		$SIMULATOR_PATH/pomdpOnlineSol $MODEL --budget 250 --simLen 25 --simNum $j --output $OUTPUT_OP --solverOP 2 --UnObsInitStateFile $INPUT
	done
	echo "" >> $SIMULATION_OUTPUT_FILE
	echo "" >> $SIMULATION_OUTPUT_FILE
		
done

### to visualize the path done by the robot
#$MODEL_PATH/simulate $MODEL_PATH/simpleHomeSwitch.png $SIMULATION_OUTPUT_FILE 5

FILEIN=simulationResults_OP_HC1x3.txt
mv $SIMULATION_OUTPUT_FILE $FILEIN

### Ploting and data process section
gnuplot << EOF

reset
set terminal pngcairo
set output "$PLOT"
set ylabel 'Reward'
set xlabel 'Duration of tree construction [ms]'
set title 'Simulation result'
set grid

set table "plotedDataHC.dat"
set print "data"
stats  "$FILEIN" using 0 nooutput

do for [i=0:STATS_blocks-1] {
   stats "$FILEIN" using 2:4 index i nooutput
   print sprintf("%e %e %e", STATS_max_x, STATS_mean_y, STATS_stddev_y*1.96/sqrt("$NRSIM"))
}
set autoscale xfix
set offsets 1,1,0,0
plot "data" using 1:2:3 w yerrorbars title "data"
unset table

plot "data" using 1:2:3 w yerrorbars title "data"

EOF

mv data plorData_OP_HC
#rm treeOP.txt
#rm simOut.txt

